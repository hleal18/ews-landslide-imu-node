#include "SoilMoisture.h"

SoilMoisture::SoilMoisture(uint8_t pin) : _pin(pin) { }

uint8_t SoilMoisture::get_soil_moisture_reading() {
    return analogRead(this->_pin);
}