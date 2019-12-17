#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>

#include "esp_sleep.h"

// UPDATE the config.h file in the same folder WITH YOUR TTN KEYS AND ADDR.
#include "config.h"
#include "IMUSensor/IMUSensor.h"

// For storing data on non-volatile memory
#include <Preferences.h>

// T-Beam specific hardwareVv
#undef BUILTIN_LED
#define BUILTIN_LED 21

enum Variable_Type
{
    ACCELERATION = 0,
    ROTATION_RATE
};

struct Sensor
{
    Variable_Type variable_type;
    uint8_t id_sensor;
};

const int epoch_size = 15;
IMUSensor imu_unit;
Axis<int8_t> acceleration_readings[epoch_size];
Axis<int8_t> rotation_rate_readings[epoch_size];
uint8_t sensor_index = 0;
Sensor sensor[2] = {{ACCELERATION, 1}, {ROTATION_RATE, 2}};

//Frequency of measures to take during a second.
//int freq_measures = 1000 / 5;
// Counter to know which measures are going to be sent
int counter = 0;

Preferences storage_manager;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

static osjob_t sendjob;
void do_send(osjob_t *j);
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 15;

void print_acceleration(Axis<int8_t> acceleration)
{
    Serial.print(acceleration.x);
    Serial.print("\t");
    Serial.print(acceleration.y);
    Serial.print("\t");
    Serial.print(acceleration.z);
    Serial.println();
}

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN, // was "14,"
    .dio = {26, 33, 32},
};

void onEvent(ev_t ev)
{
    switch (ev)
    {
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        digitalWrite(BUILTIN_LED, LOW);
        // Schedule next transmission
        esp_sleep_enable_timer_wakeup(TX_INTERVAL * 1000000);
        esp_deep_sleep_start();
        //ESP.restart();
        //delay(TX_INTERVAL * 1000);
        do_send(&sendjob);
        break;
    }
}

void do_send(osjob_t *j)
{
    Serial.print("Variable type: ");
    Serial.println(sensor[sensor_index].variable_type);

    Axis<double> temp_axis;

    if (sensor[sensor_index].variable_type == ACCELERATION)
    {
        for (int i = 0; i < epoch_size; i++)
        {
            temp_axis = imu_unit.get_acceleration();
            acceleration_readings[i].x = temp_axis.x;
            acceleration_readings[i].y = temp_axis.y;
            acceleration_readings[i].z = temp_axis.z;
            print_acceleration(acceleration_readings[i]);
        }
    }
    else if (sensor[sensor_index].variable_type == ROTATION_RATE)
    {
        for (int i = 0; i < epoch_size; i++)
        {
            temp_axis = imu_unit.get_rotation_rate();
            rotation_rate_readings[i].x = temp_axis.x;
            rotation_rate_readings[i].y = temp_axis.y;
            rotation_rate_readings[i].z = temp_axis.z;
            print_acceleration(rotation_rate_readings[i]);
        }
    }
    counter++;

    uint8_t arrBuff[epoch_size * 3 + 2];

    arrBuff[0] = sensor[sensor_index].id_sensor;
    arrBuff[1] = sensor[sensor_index].variable_type;

    for (int i = 2, accind = 0; i < epoch_size * 3 + 2; i += 3, accind++)
    {
        if (sensor[sensor_index].variable_type == ACCELERATION)
        {
            arrBuff[i] = acceleration_readings[accind].x;
            arrBuff[i + 1] = acceleration_readings[accind].y;
            arrBuff[i + 2] = acceleration_readings[accind].z;
        }
        else if (sensor[sensor_index].variable_type == ROTATION_RATE)
        {
            arrBuff[i] = rotation_rate_readings[accind].x;
            arrBuff[i + 1] = rotation_rate_readings[accind].y;
            arrBuff[i + 2] = rotation_rate_readings[accind].z;
        }
    }

    LMIC_setTxData2(1, arrBuff, sizeof(arrBuff), 0);
    Serial.println(F("Packet queued"));
    digitalWrite(BUILTIN_LED, HIGH);

    sensor_index = (sensor_index == 1) ? 0 : 1;
    storage_manager.putUInt("sensor_index", sensor_index);
    storage_manager.end();
}

void setup()
{
    Serial.begin(9600);

    //Turn off WiFi and Bluetooth
    WiFi.mode(WIFI_OFF);
    btStop();
    imu_unit.begin();

    // Starting storage functions.
    storage_manager.begin("variables_state", false);
    sensor_index = storage_manager.getUInt("sensor_index", 0);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);

    LMIC_setupChannel(0, 903900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(1, 904100000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
    LMIC_setupChannel(2, 904300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(3, 904500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(4, 904700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(5, 904900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(6, 905100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(7, 905300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(8, 904600000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF10, 14);

    do_send(&sendjob);
    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED, LOW);
}

void loop()
{
    os_runloop_once();
}