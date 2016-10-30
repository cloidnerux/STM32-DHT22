
#pragma once
#include "system.h"

#include "stm32l1xx_conf.h"

enum
{
  DHT22_ERROR_NOERROR = 0,
  DHT22_ERROR_TIMEOUT = 1
};

class DHT22
{
private:

public:
    DHT22();
    void StartReading();
    float GetHumidity();
    float GetTemperature();
};
