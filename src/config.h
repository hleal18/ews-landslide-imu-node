#ifndef LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED
#define LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED

// UPDATE WITH YOUR TTN KEYS AND ADDR.
// ABP
static PROGMEM u1_t NWKSKEY[16] = {0x92, 0x2D, 0xAD, 0x1D, 0x2F, 0xDE, 0x0F, 0xA3, 0xEF, 0x6C, 0x45, 0xF7, 0xB1, 0x97, 0xFA, 0x6F}; // LoRaWAN NwkSKey, network session key
static u1_t PROGMEM APPSKEY[16] = {0xC5, 0x8E, 0x16, 0x14, 0xFD, 0x58, 0x96, 0xDD, 0x35, 0x49, 0x61, 0x65, 0xB6, 0xEC, 0x24, 0xFE}; // LoRaWAN AppSKey, application session key
static const u4_t DEVADDR = 0x26021925;                                                                                             // LoRaWAN end-device address (DevAddr)

#endif //LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED