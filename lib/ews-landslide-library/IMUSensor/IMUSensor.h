#ifndef IMUSENSOR_H
#define IMUSENSOR_H

#include <MPU9255.h>

template<typename T>
struct Axis {
    T x;
    T y;
    T z;

    Axis() : x(0), y(0), z(0) {}
};

class IMUSensor
{
public:
    IMUSensor();
    IMUSensor(const uint8_t sda, const uint8_t scl, const scales acceleration_scale, const scales gyroscope_scale, const double magnetometer_scale);

    bool begin();

    Axis<int16_t> get_accelerometer_readings();
    Axis<int16_t> get_gyroscope_readings();
    Axis<int16_t> get_magnetometer_readings();
    int16_t get_pressure_sensor_readings();
    
    void set_accelerometer_scale(scales accelerometer_scale);

    Axis<double> get_acceleration();
    Axis<double> get_rotation_rate();
    Axis<double> get_magnetic_flux();

private:
    const float _G = 9.81;

    MPU9255 _mpu;
    //BMP180 _pressure_sensor;
    
    uint8_t _sda;
    uint8_t _scl;   

    scales _accelerometer_scale;
    scales _gyroscope_scale;
    double _magnetometer_scale;

    double calculate_acceleration(const int16_t raw);
    double calculate_rotation_rate(const int16_t raw);
    double calculate_magnetic_flux(const int16_t raw);
};

#endif