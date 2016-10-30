
#include "DHT22.h"
#include "stm32l1xx_conf.h"


#define CC2IF (1<<2)
#define CC3IF (1<<3)

/// The DHT22 is connected to D15 on a NUCLEO152RE, which is GPIOB pin 8
/// This corresponds to timer 4 channel 3

void HandleTimerInterrupt();
void HandleDmaInterrupt(struct __DMA_HandleTypeDef * hdma);

/// States for the interrupt driven state machine
enum STATES
{
    STATE_START = 0,
    STATE_RESPONSE1,  //First falling flank
    STATE_RESPONSE2,  //second falling flank
    STATE_DATA,
    STATE_IDLE,
    STATE_ERROR
};

/// The state variable for the interrupt driven state machine
volatile int state = STATE_START;

/// Global init structures to be accessed from the interrupts
GPIO_InitTypeDef        GPIO_InitStructure;
TIM_TimeBaseInitTypeDef TIM_InitStructure;
TIM_OCInitTypeDef       TIM_OCInitStructure;
TIM_ICInitTypeDef       TIM_ICInitStructure;
DMA_InitTypeDef         DMA_InitStructure;
///////////////////////////////////////////////////

/// The time data received
uint32_t dht22Data[40];

/// The temperature received
volatile uint32_t mTemperature;
/// The humidity received
volatile uint32_t mHumidity;

uint32_t _t1, _t2;

/// Initialize the needed resources
DHT22::DHT22()
{
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  NVIC_InitTypeDef nvicStructure;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);  //Activate alternate function

  GPIOB->ODR &= ~GPIO_Pin_8;

  TIM_TimeBaseStructInit(&TIM_InitStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_ICStructInit(&TIM_ICInitStructure);

  TIM_InitStructure.TIM_Prescaler = (SystemCoreClock / 1000000); ///1탎 per clock
  TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_InitStructure.TIM_Period = 32000;
  TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM4, &TIM_InitStructure);
  TIM_Cmd(TIM4, ENABLE);
  TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
  TIM_ITConfig(TIM4, TIM_IT_CC3, DISABLE);  //No capture interrupts yet
  TIM_ITConfig(TIM4, TIM_IT_CC2, DISABLE);


  nvicStructure.NVIC_IRQChannel = TIM4_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);

  nvicStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);

  DMA_DeInit(DMA1_Channel5);

  DMA_StructInit(&DMA_InitStructure);


  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM4->CCR3);
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&dht22Data[0];
  DMA_InitStructure.DMA_BufferSize = 40;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_InitStructure.DMA_Priority = 1;

  DMA_Init(DMA1_Channel5, &DMA_InitStructure);

}

/// Trigger the reading of the DHT22 data, this is a non blocking call
void DHT22::StartReading()
{
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_ICStructInit(&TIM_ICInitStructure);

  TIM_ITConfig(TIM4, TIM_IT_Update | TIM_IT_CC2 | TIM_IT_CC3, DISABLE);
  TIM_ClearITPendingBit(TIM4, TIM_IT_Update | TIM_IT_CC2 | TIM_IT_CC3);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_8);

  DMA_Cmd(DMA1_Channel5, DISABLE);
  DMA_DeInit(DMA1_Channel5);

  TIM_DeInit(TIM4);
  TIM_InitStructure.TIM_Prescaler = (SystemCoreClock / 1000000) - 1; //1탎 per clock
  TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_InitStructure.TIM_Period = 10000;    //10ms timeout
  TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM4, &TIM_InitStructure);
  TIM_Cmd(TIM4, ENABLE);


  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Active;
  TIM_OCInitStructure.TIM_Pulse = 1000;   //1ms
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICFilter = 0;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;

  TIM_ICInit(TIM4, &TIM_ICInitStructure);

  TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
  TIM4->CNT = 0;
  TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM4, TIM_IT_CC3, DISABLE);  //No capture interrupts yet
  TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);

  DMA1_Channel5->CNDTR = 40;
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);
  DMA_ClearITPendingBit(DMA1_IT_TC5);
  DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, DISABLE);

  state = STATE_START;
}

/// needs float support in compiler and linker settings!
/// returns the last gathered humidity
float DHT22::GetHumidity()
{
    return ((float)mHumidity)/10.0f;
}

/// needs float support in compiler and linker settings!
/// returns the last gathered temperature
float DHT22::GetTemperature()
{
    printf("Temperature: %d.%d \r\n", mTemperature/10, mTemperature%10);
    return ((float)mTemperature)/10.0f;
}

/// General function to handle an error state and reset all peripherals
/// Uncomment if printf is available and the debug output is needed
void HandleReadError(bool wrongState)
{
  if(wrongState)
  {
    //printf("Wrong state: ");
  }
  if(state == STATE_DATA)
  {
    //printf("DMA transfer timeout, number of Transfers: %d\r\n", DMA_GetCurrDataCounter(DMA1_Channel5));
    if(DMA_GetFlagStatus(DMA1_FLAG_TE5) != RESET)
    {
      //printf("DMA Error!\r\n");
      DMA_ClearFlag(DMA1_FLAG_TE5);
    }
    if(TIM_GetFlagStatus(TIM4, TIM_FLAG_CC3OF) != RESET)
    {
      //printf("Timer overcapture flag!\r\n");
      TIM_GetCapture3(TIM4);
    }
  }
  else if(state == STATE_RESPONSE1)
  {
    //printf("First Response timeout\r\n");
  }
  else if(state == STATE_RESPONSE2)
  {
    //printf("Second Response timeout\r\n");
  }
  else if(state == STATE_START)
  {
    //printf("Start Timeout\r\n");
  }
  else
  {
    //printf("Other timeout with state %d\r\n", state);
  }
  //printf("T1: %x T2: %x\r\n", _t1, _t2);
  TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
  TIM_ITConfig(TIM4, TIM_IT_Update | TIM_IT_CC2 | TIM_IT_CC3 , DISABLE);
  DMA_Cmd(DMA1_Channel5, DISABLE);

  DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, DISABLE);
  ///maybe set an error flag here
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_8);
}

extern "C" void TIM4_IRQHandler()
{
    if(TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
        if(state == STATE_START)    //We waited the 1ms time for start signal
        {
            GPIO_SetBits(GPIOB, GPIO_Pin_8);
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
            GPIO_Init(GPIOB, &GPIO_InitStructure);

            TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
            TIM_SetCounter(TIM4, 0);
            TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
            TIM_ITConfig(TIM4, TIM_IT_CC2, DISABLE);

            TIM_GetCapture3(TIM4);

            TIM_ITConfig(TIM4, TIM_IT_Update | TIM_IT_CC3, ENABLE);   //Enable capture intterupt
            state = STATE_RESPONSE1;
        }
        else        //There is something wrong here
        {
            HandleReadError(true);
        }
    }
    if(TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
        if(state == STATE_RESPONSE1)
        {
            _t1 = TIM_GetCapture3(TIM4);
            if(_t1 > 50)    //First falling flank is maximum 40탎
            {
                //Error on the response signal
                HandleReadError(false);
                state = STATE_IDLE;
                return;
            }
            else
            {
              state = STATE_RESPONSE2;
              TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
              TIM4->CNT = 0;
              TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
              TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
            }
        }
        else if(state == STATE_RESPONSE2)
        {
          _t2 = TIM_GetCapture3(TIM4);
          if(_t2 > 200)   //Second falling flank is maximum 160탎
          {
            HandleReadError(false);
            state = STATE_IDLE;
            return;
          }
          else
          {
            state = STATE_DATA;
            TIM_ITConfig(TIM4, TIM_IT_CC3, DISABLE);

            DMA_Cmd(DMA1_Channel5, ENABLE);
            TIM_DMAConfig(TIM4, TIM_DMABase_CCR3, TIM_DMABurstLength_1Transfer);
            TIM_DMACmd(TIM4, TIM_DMA_CC3, ENABLE);

            TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
            TIM4->CNT = 0;
            TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
            TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
            DMA_ClearITPendingBit(DMA1_IT_TC5);
            DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
            ///Trigger the DMA reading
          }
        }
        else
        {
          HandleReadError(true);
          state = STATE_ERROR;
          SystemParameter.dht22ReadError = DHT22_ERROR_TIMEOUT;
        }
    }
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
      /*if(state == STATE_IDLE)
      {
        printf("Timeout, but data was read!\r\n");
      }
      else
      {
        printf("Timeout in dht22 readout\r\n");
      }*/
      HandleReadError(false);
    }
}

extern "C" void DMA1_Channel5_IRQHandler()
{
    DMA_ClearITPendingBit(DMA1_IT_TC5);
    if(state != STATE_DATA)
    {
      //printf("DMA Interrupt with state: %d", state);
      HandleReadError(true);
      return;
    }
    //We finished the data aquisition
    state = STATE_IDLE;
    uint8_t bytes[5], checksum;
    bytes[0] = (dht22Data[0] > 100)?(1):(0);
    for(int i = 1; i <40; i++)
    {
        bytes[i/8] <<= 1;
        bytes[i/8] |= ((dht22Data[i]-dht22Data[i-1]) > 100)?(1):(0);      //Saved length should be 50탎 + 26탎 for a 0 and 50탎 + 70탎 for a one
    }
    checksum = (bytes[0] + bytes[1] + bytes[2] + bytes[3]);
    if(bytes[4] != checksum) //Checksum error
    {
        //printf("Checksum error %02x %02x %02x %02x %02x\r\n", bytes[0], bytes[1], bytes[2], bytes[3], bytes[4]);
        return;
    }
    mTemperature = (bytes[2] << 8) | bytes[3];
    mHumidity = (bytes[0] << 8) | bytes[1];

    DMA_Cmd(DMA1_Channel5, DISABLE);
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, DISABLE);
    TIM_ITConfig(TIM4, TIM_IT_CC3, DISABLE);
    TIM_ITConfig(TIM4, TIM_IT_CC2, DISABLE);
    TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_8);
}

