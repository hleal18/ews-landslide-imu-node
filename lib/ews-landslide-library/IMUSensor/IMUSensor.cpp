#include "IMUSensor.h"

IMUSensor::IMUSensor() : _mpu(), _sda(21), _scl(22)
//, _pressure_sensor()
{
    _accelerometer_scale = scale_2g;
    _gyroscope_scale = scale_250dps;
    _magnetometer_scale = 0.06;
}

IMUSensor::IMUSensor(const uint8_t sda, const uint8_t scl, const scales acceleration_scale, const scales gyroscope_scale, const double magnetometer_scale)
    : _mpu(), _sda(sda), _scl(scl)
//, _pressure_sensor()
{
    _accelerometer_scale = acceleration_scale;
    _gyroscope_scale = gyroscope_scale;
    _magnetometer_scale = magnetometer_scale;
}

bool IMUSensor::begin()
{
    return _mpu.init();
}

Axis<int16_t> IMUSensor::get_accelerometer_readings()
{
    Axis<int16_t> raw_acceleration;

    _mpu.read_acc();

    raw_acceleration.x = _mpu.ax;
    raw_acceleration.y = _mpu.ay;
    raw_acceleration.z = _mpu.az;

    return raw_acceleration;
}

Axis<int16_t> IMUSensor::get_gyroscope_readings()
{
    Axis<int16_t> raw_rotation_rate;

    _mpu.read_gyro();

    raw_rotation_rate.x = _mpu.gx;
    raw_rotation_rate.y = _mpu.gy;
    raw_rotation_rate.z = _mpu.gz;

    return raw_rotation_rate;
}

Axis<int16_t> IMUSensor::get_magnetometer_readings()
{
    Axis<int16_t> raw_magnetic_flux;

    _mpu.read_mag();

    raw_magnetic_flux.x = _mpu.mx;
    raw_magnetic_flux.y = _mpu.my;
    raw_magnetic_flux.z = _mpu.mz;

    return raw_magnetic_flux;
}

Axis<double> IMUSensor::get_acceleration()
{
    Axis<double> acceleration;
    Axis<int16_t> raw_acceleration = this->get_accelerometer_readings();

    acceleration.x = this->calculate_acceleration(raw_acceleration.x);
    acceleration.y = this->calculate_acceleration(raw_acceleration.y);
    acceleration.z = this->calculate_acceleration(raw_acceleration.z);

    return acceleration;
}

void IMUSensor::set_accelerometer_scale(scales accelerometer_scale) {
    _accelerometer_scale = accelerometer_scale;
    _mpu.set_acc_scale(accelerometer_scale);
}

Axis<double> IMUSensor::get_rotation_rate()
{
    Axis<double> rotation_rate;
    Axis<int16_t> raw_rotation_rate = this->get_gyroscope_readings();

    rotation_rate.x = this->calculate_rotation_rate(raw_rotation_rate.x);
    rotation_rate.y = this->calculate_rotation_rate(raw_rotation_rate.y);
    rotation_rate.z = this->calculate_rotation_rate(raw_rotation_rate.z);

    return rotation_rate;
}

Axis<double> IMUSensor::get_magnetic_flux()
{
    Axis<double> magnetic_flux;
    Axis<int16_t> raw_magnetic_flux = this->get_magnetometer_readings();

    magnetic_flux.x = this->calculate_magnetic_flux(raw_magnetic_flux.x);
    magnetic_flux.y = this->calculate_magnetic_flux(raw_magnetic_flux.y);
    magnetic_flux.z = this->calculate_magnetic_flux(raw_magnetic_flux.z);

    return magnetic_flux;
}

double IMUSensor::calculate_acceleration(const int16_t raw)
{
    double axis_acceleration = 1;
    
    if (_accelerometer_scale == scale_2g)
    {
        axis_acceleration = raw / 16384.0;
    }
    else if (_accelerometer_scale == scale_4g)
    {
        axis_acceleration = raw / 8192.0;
    }
    else if (_accelerometer_scale == scale_8g)
    {
        axis_acceleration = raw / 4096.0;
    }
    else if (_accelerometer_scale == scale_16g)
    {
        axis_acceleration = raw / 2048.0;
    }

    return axis_acceleration * _G;
}

double IMUSensor::calculate_rotation_rate(const int16_t raw)
{
    double axis_rotation_rate = 1;

    if (_gyroscope_scale == scale_250dps)
    {
        axis_rotation_rate = raw / 131;
    }
    else if (_gyroscope_scale == scale_500dps)
    {
        axis_rotation_rate = raw / 65.5;
    }
    else if (_gyroscope_scale == scale_1000dps)
    {
        axis_rotation_rate = raw / 32.8;
    }
    else if (_gyroscope_scale == scale_2000dps)
    {
        axis_rotation_rate = raw / 16.4;
    }

    return axis_rotation_rate;
}

double IMUSensor::calculate_magnetic_flux(const int16_t raw)
{
    // It lacks sensitivity parameter from example sketch.
    return (raw * _magnetometer_scale) / 0.6;
}