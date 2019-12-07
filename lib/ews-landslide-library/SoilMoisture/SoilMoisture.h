#ifndef SOILMOISTURE_H
#define SOILMOISTURE_H

#include <Arduino.h>

class SoilMoisture
{
public:
    SoilMoisture(uint8_t pin);
    uint8_t get_soil_moisture_reading();

private:
    uint8_t _pin;
};

#endif