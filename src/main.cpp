#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>

#include "esp_sleep.h"

// UPDATE the config.h file in the same folder WITH YOUR TTN KEYS AND ADDR.
#include "config.h"
#include "IMUSensor/IMUSensor.h"

// T-Beam specific hardwareVv
#undef BUILTIN_LED
#define BUILTIN_LED 21

const int acceleration_size = 8;
IMUSensor magia;
Axis<int16_t> acceleration_readings[acceleration_size];
// Type 0 = 'acceleration' on backend servers.
int variable_type = 0;
//Frequency of measures to take during a second.
int freq_measures = 1000/5;
// Counter to know which measures are going to be sent
int counter = 0;
long last_time = 0;
int last_index = 0;

union Convert {
    int bytes[sizeof(double)];
    double real;
};

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

static osjob_t sendjob;
void do_send(osjob_t *j);
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 1;

void print_acceleration(Axis<int16_t> acceleration)
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
        esp_sleep_enable_timer_wakeup(TX_INTERVAL*1000000);
        esp_deep_sleep_start();
        //delay(TX_INTERVAL * 1000);
        do_send(&sendjob);
        break;
    }
}

void do_send(osjob_t *j)
{

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        acceleration_readings[0] = magia.get_accelerometer_readings();
        Serial.print("Sending: ");
        print_acceleration(acceleration_readings[0]);
        
        
        uint8_t arrBuff[acceleration_size * 6 + 1];
        
        arrBuff[0] = variable_type;
        
        for(int i = 1, accind = 0; i < acceleration_size*6 + 1; i+= 6, accind++) {
            arrBuff[i] = acceleration_readings[accind].x >> 8;
            arrBuff[i+1] = acceleration_readings[accind].x;
            arrBuff[i+2] = acceleration_readings[accind].y >> 8;
            arrBuff[i+3] = acceleration_readings[accind].y;
            arrBuff[i+4] = acceleration_readings[accind].z >> 8;
            arrBuff[i+5] = acceleration_readings[accind].z;
        }
        
        Serial.print("Counter: ");
        Serial.println(counter);
        
        LMIC_setTxData2(1, arrBuff, sizeof(arrBuff), 0);
        Serial.println(F("Packet queued"));
        digitalWrite(BUILTIN_LED, HIGH);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup()
{
    Serial.begin(9600);
    Serial.println(F("TTN Mapper"));

    //Turn off WiFi and Bluetooth
    WiFi.mode(WIFI_OFF);
    btStop();
    magia.begin();

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
    
    last_time = millis();
}

void loop()
{
    acceleration_readings[last_index++] = magia.get_accelerometer_readings();
    //print_acceleration(acceleration_readings[last_index-1]);
    counter++;
    if (last_index >= acceleration_size) last_index = 0;
    
    while(millis() - last_time < freq_measures) {
        
    }
    last_time = millis();
    os_runloop_once();
}