#include <Arduino.h>

//Nachricht für Armaturenbeleuchtung einschalten
unsigned char DASHBOARD_LIGHTING_ON[2] = {0xFD, 0x00};
//Nachricht für Armaturenbeleuchtung auszuschalten
unsigned char DASHBOARD_LIGHTING_OFF[2] = {0xFE, 0x00};

//Nachricht für Keepalive
unsigned char IDRIVE_CTRL_KEEPALIVE_KCAN2[8] = {0x00, 0x00, 0x00, 0x00, 0x57, 0x2F, 0x00, 0x60};

//Nachricht für Keepalive (KCAN)
unsigned char IDRIVE_CTRL_KEEPALIVE[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//Nachricht für Rotary Init
unsigned char IDRIVE_CTRL_INIT[8] = {0x1D, 0xE1, 0x0, 0xF0, 0xFF, 0x7F, 0xDE, 0x4};