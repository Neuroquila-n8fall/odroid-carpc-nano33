#include <Arduino.h>

//Adresse für Armaturenbeleuchtung
//Nachricht: Länge 2
//Byte 0: Intensität
//Byte 1: 0x0
//Mögliche Werte:
//Dimmwert    byte[0]
//0           0
//1           28
//2           56
//3           84
//4           112
//5           141
//6           169
//7           197
//8           225
//9           253
const unsigned long DASHBOARD_LIGHTING_ADDR = 0x202;

//Adresse des iDrive Controllers für Buttons & Joystick
const int IDRIVE_CTRL_BTN_ADDR = 0x267;
//Adresse des iDrive Controllers für Rotation
const int IDRIVE_CTRL_ROT_ADDR = 0x264;
//Adresse für Status des iDrive Controllers. byte[5] = Counter, byte[6] Initialisiert: 1 = true, 6 = false
const int IDRIVE_CTRL_STATUS_ADDR = 0x5E7;

//Falls ich mal auf den Trichter komme auf den Touch Controller vom F-Modell zu wechseln...
const int IDRIVE_CTRL_KEEPALIVE_ADDR_KCAN2 = 0x560;

//Keep-Alive Nachricht, damit der Controller aufgeweckt bleibt
//Quelladresse für Keepalive (ebenfalls KCAN2)
const int IDRIVE_CTRL_KEEPALIVE_ADDR = 0x501;

//Initialisierung des iDrive Controllers, damit dieser die Drehung übermittelt
//Quelladresse für Rotary-Init
const int IDRIVE_CTRL_INIT_ADDR = 0x273;
//Nachrichtenadresse erfolgreiche Rotary-Initialisierung
const int IDRIVE_CTRL_INIT_RESPONSE_ADDR = 0x277;

//Datum und Uhrzeit vom Kombiinstrument (Tacho-Cluster)
const int KOMBI_DATETIME_ADDR = 0x2F8;

//MFL (Multifunktionslenkrad)
const int MFL_BUTTON_ADDR = 0x1D6;

//CAS (Car Access System, Schlüsselstatus und Zündung)
const int CAS_ADDR = 0x130;

//CAS ZV (Zentralverriegelung, Funkschlüssel)
const int CAS_ZV_ADDR = 0x23A;

//CIC (Car Information Computer)
const int CIC_ADDR = 0x273;

//RLS (Regen-Licht-Sensorik)
const int RLS_ADDR = 0x314;

//IHKA (Innenraum Heizung und Klimaanlage) - Status innenraumsensorik
const int IHKA_ADDR = 0x32E;

//LM (Lichtmodul) Dimmer für Armaturenbeleuchtung bei Abblendlicht AKTIV
const int LM_DIM_ADDR = 0x202;

//Rückspiegel Abblendfunktion
const int SPIEGEL_ABBLEND_ADDR = 0x286;

//PDC Abstandsmeldung 1
const int PDC_ABSTANDSMELDUNG_1_ADDR = 0x1C2;

//PDC Abstandsmeldung 2
const int PDC_ABSTANDSMELDUNG_2_ADDR = 0x1C3;

//PDC Akustikmeldung
const int PDC_AKUSTIKMELDUNG_ADDR = 0x1C6;

//LM Status Rückwärtsgang
const int LM_REVERSESTATUS_ADDR = 0x3B0;

//DME - DDE Powermanagement Batterspiespannung
const int DMEDDE_POWERMGMT_BATTERY_ADDR = 0x3B4;

