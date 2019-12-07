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



// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

static osjob_t sendjob;
void do_send(osjob_t *j);
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 5;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN, // was "14,"
    .dio = {26, 33, 32},
};

IMUSensor magia;

void onEvent(ev_t ev)
{
    switch (ev)
    {
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        digitalWrite(BUILTIN_LED, LOW);
        // Schedule next transmission
        //esp_sleep_enable_timer_wakeup(TX_INTERVAL*1000000);
        //esp_deep_sleep_start();
        delay(TX_INTERVAL * 1000);
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

        // Prepare upstream data transmission at the next possible time.
        uint8_t sensor_type = 0;
        uint16_t value = 2500;
        Axis<double> acceleration = magia.get_acceleration();
        Serial.print("X: ");
        Serial.println(acceleration.x);

        uint8_t arrBuff[3] = {sensor_type, value >> 8, value};

        // uint8_t arrBuff[5] = {sensor_type,
        //     acceleration.x >> 24,
        //     acceleration.x >> 16,
        //     acceleration.x >> 8,
        //     acceleration.x
        // };

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
    LMIC_setDrTxpow(DR_SF12, 14);

    do_send(&sendjob);
    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED, LOW);
}

void loop()
{
    os_runloop_once();
}