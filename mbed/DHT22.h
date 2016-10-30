
#ifndef MBED_DHT22_H
#define MBED_DHT22_H

#include "mbed.h"


#define DHT22_ERROR_VALUE -99.5

typedef enum {
    DHT_ERROR_NONE = 0,
    DHT_BUS_HUNG,
    DHT_ERROR_NOT_PRESENT,
    DHT_ERROR_ACK_TOO_LONG,
    DHT_ERROR_SYNC_TIMEOUT,
    DHT_ERROR_DATA_TIMEOUT,
    DHT_ERROR_CHECKSUM,
    DHT_ERROR_TOOQUICK
} DHT22_ERROR;

class DHT22 {
private:
    uint32_t  _lastReadTime;
    PinName _data;
    float   _lastHumidity;
    float   _lastTemperature;
public:
    DHT22(PinName Data);
    ~DHT22();
    DHT22_ERROR readData(void);
    float getHumidity();
    float getTemperatureC();
    void clockReset();
};

#endif /*_DHT22_H_*/
