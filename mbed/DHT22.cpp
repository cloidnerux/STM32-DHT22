
#include "DHT22.h"
#include "system.h"


///Initialize the pins and variables
///WARNIGN: Even if there is a pin given as an argument here, the library can and will not work without
///adaption, as input capture is used to get the data.
///For your system you have to look up which timer resource can track an input capture on the corresponding
///pin, then change the timer and GPIO references to the correct ones!
DHT22::DHT22(PinName Data) {

    _data = Data;                // Set Data Pin
    _lastReadTime = 0;
    _lastHumidity = 0;
    _lastTemperature = DHT22_ERROR_VALUE;
}


///destructor, nothing to do here
DHT22::~DHT22() {
}


/// Blocking call, get the data out of a DHT22
/// The DHT22 is quite simple. The data is transmitted over a bidirectional line which has a pull-up resistor
/// The data transfer is initiated by a >1ms low pulse on the line we have to create.
/// After this the DHT22 will hold down the line 80us and then 80us up as a start signal
/// Then the data is transmitted in a pulse width modulation. After 50탎 of a low signal the data bus is held high
/// for ~ 26탎 for a '0' and ~ 70탎 for a high
/// This function will initiate the transmission and will capture the 40 bits with the timer input capture
/// In my case, the sensor is hooked up onto D15 of a NUCLEO152RE, which is timer4 input capture 3
/// The mbed libraries are not well suited for the switchover from output to AF input, therefore the HAL is used
/// The timer is configured to tick every 탎, which is a prescaler of 24 for the 24MHz MBED clock.
/// This has to be adapted to your system clock frequency.
/// The timer is not only used for the input capture, but also to give a timeout.
DHT22_ERROR DHT22::readData() {
  TIM_Base_InitTypeDef    timerInitStructure;
  TIM_HandleTypeDef       _t;
  TIM_IC_InitTypeDef      TIM_ICInitStructure;
  GPIO_InitTypeDef        GPIO_InitStructure;
  __TIM4_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  uint32_t data[40];
  uint32_t last;

  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStructure.Alternate= GPIO_AF2_TIM4;
  GPIO_InitStructure.Pin= GPIO_PIN_8;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;

  timerInitStructure.Prescaler = 24; //1탎 per clock
  timerInitStructure.CounterMode = TIM_COUNTERMODE_UP;
  timerInitStructure.Period = 32000;
  timerInitStructure.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  _t.Instance = TIM4;
  _t.Init = timerInitStructure;
  HAL_TIM_Base_Init(&_t);
  HAL_TIM_Base_Start(&_t);

  TIM_ICInitStructure.ICFilter= 0;
  TIM_ICInitStructure.ICPolarity= TIM_ICPOLARITY_FALLING;
  TIM_ICInitStructure.ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.ICSelection= TIM_ICSELECTION_DIRECTTI;
  _t.Channel = HAL_TIM_ACTIVE_CHANNEL_3;
  HAL_TIM_IC_ConfigChannel(&_t, &TIM_ICInitStructure, TIM_CHANNEL_3);
  HAL_TIM_IC_Init(&_t);

  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
  wait_ms(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

  GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  wait_us(20);
  __HAL_TIM_SetCounter(&_t, 0);

  HAL_TIM_IC_Start(&_t, TIM_CHANNEL_3);
  while(__HAL_TIM_GET_FLAG(&_t, TIM_FLAG_CC3) == 0) //wait for capture
  {
    if(__HAL_TIM_GetCounter(&_t) > 4500)              //timeout
    {
      return DHT_BUS_HUNG;
    }
  }
  if(__HAL_TIM_GetCompare(&_t, TIM_CHANNEL_3) > 200 )
  {
    HAL_TIM_IC_Stop(&_t, TIM_CHANNEL_3);
    return DHT_ERROR_NOT_PRESENT;
  }
  __HAL_TIM_SetCounter(&_t, 0);
  for(int i = 0; i < 40; i++)
  {
    while(__HAL_TIM_GET_FLAG(&_t, TIM_FLAG_CC3) == 0) //wait for capture
    {
      if(__HAL_TIM_GetCounter(&_t) > 4500)              //timeout
      {
        return DHT_ERROR_DATA_TIMEOUT;
      }
    }
    data[i] = __HAL_TIM_GetCompare(&_t, TIM_CHANNEL_3);
  }
  HAL_TIM_IC_Stop(&_t, TIM_CHANNEL_3);

  uint8_t bytes[5], checksum;
  bytes[0] = (data[0] > 100)?(1):(0);
  for(int i = 1; i <40; i++)
  {
      bytes[i/8] <<= 1;
      bytes[i/8] |= ((data[i]-data[i-1]) > 100)?(1):(0);      //Saved length should be 50탎 + 26탎 for a 0 and 50탎 + 70탎 for a one
  }
  checksum = (bytes[0] + bytes[1] + bytes[2] + bytes[3]);
  if(bytes[4] != checksum) //Checksum error
  {
      return DHT_ERROR_CHECKSUM;
  }
  _lastTemperature = (bytes[2] << 8) | bytes[3];
  _lastHumidity = (bytes[0] << 8) | bytes[1];
  return DHT_ERROR_NONE;
}

float DHT22::getTemperatureC() {
    return _lastTemperature / 10.0f;
}

float DHT22::getHumidity() {
    return _lastHumidity / 10.0f;
}
