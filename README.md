# STM32-DHT22
A library to read DHT22 temperature and humidity sensors on a STM32 plattform using input capture, interrupts and DMA

There are two version available. In the SPL folder is a version solely based upon the SPL and using DMA to transfer the data. The complete transfer is asynchronous and non blocking.
The second version is based upon the mbed system and the HAL. This is blocking, but also uses input capture to gather the data. 
Both versions have a timeout and will terminate.

This library is not portable, as it has to be adapted to the available hardware resources.
In my case the DHT22 is connected to the D15 pin on a Nucleo152RE, which is GPIOB pin 8 or timer 4 channel 3. An external pull-up resistor with 4k7 is used.
To adapt the library to your system check which timer can capture on that pin with the datasheet of your used STM32 mcu. Then adapt the GPIO, timer and DMA settings. For the timer adapt the presacler to your system core clock. Be careful to check if the backend system you have uses the timer you want to use already, as this will produce unpredicted behavior.

