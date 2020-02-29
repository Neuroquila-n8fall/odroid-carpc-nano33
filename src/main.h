#pragma once
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <mcp_can.h>
#include <BPLib.h>

#include <Adafruit_INA219.h>

#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include "arduino_secrets.h"
#include <ArduinoBLE.h>
#include <RTCZero.h>

/* ----------------- PWM, FAN */

//Maximaler dutycycle also frequenz. 1920 entspricht 100%DC @ 25kHz, da 48.000.000 / 25.000 = 1920
const int MAX_FANDUTY = 1920;

//Minimaler cycle.
const int MIN_FANDUTY = 0;

//PWM Setup
void setupPWM();
//Set Duty Cycle
void setFanDutyCycle(int pcCycle);

/* ----------------- OLED */
//Initialize the display.
void setupOled();

/* ---------------- RTC, Connectivity */
//Sets up and connects wifi according to the constants defined in "arduino_secrets.h" as "SECRET_SSID" and "SECRET_PASS"
//This function throttles itself so it only tries to connect after a certain delay defined in "connect_delay"
void setupWifi();
//Setup UDP Connectivity
void setupUDP();
//Connects to a NTP server via UDP and retrieves the current time.
void updateTime();
//RTC Init
void setupRTC();
//Timestamp builder
void buildtimeStamp();
//Send NTP Packet
unsigned long sendNTPpacket(IPAddress& address);

// local port to listen for UDP packets
unsigned int localPort = 2390;
// de.pool.ntp.org NTP server 88.198.17.248
IPAddress timeServer(88, 198, 17, 248);
// NTP time stamp is in the first 48 bytes of the message
const int NTP_PACKET_SIZE = 48;
//buffer to hold incoming and outgoing packets
byte packetBuffer[NTP_PACKET_SIZE];
// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

//Stores the result of a connection attempt.
int wifiStatus = WL_IDLE_STATUS;
//Seconds of delay between retry attempts
const int connect_delay = 10;
//Timer
unsigned long previousConnectionAttempt = 0;

//RTC
RTCZero rtc; 

/* -------------------- Voltage Monitoring */
Adafruit_INA219 ina219;
//Establish connection with the INA219 chip and initialize it
void setupIna();
//Retrieves all the data and fills it into the variables:
//float shuntvoltage = Reads the voltage between V- and V+. This is the measured voltage drop across the shunt resistor.
//float busvoltage = Reads the voltage between GND and V-. This is the total voltage seen by the circuit under test. (Supply voltage -shunt voltage).
//float Reads the current, derived via Ohms Law from the measured shunt voltage
//float loadvoltage = bus voltage while also taking into account the shunt voltage
//float power_mW = power
void checkIna();

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;

/* Temperature */
const int ONE_WIRE_BUS = A0;    //DATA PORT DALLAS 18B50
const int TEMP_RESOLUTION = 11; //TEMPERATURE RESOLUTION

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress firstDeviceAddress;

float temperature = 0.0;
float rawtemp = 0.0;

//Setup sensors
void setupTemperatureSensors();
//retrieve the current temperature of sensor
float checkTemperature();

/*------------- Functions -------------*/
//Status der Zündung abrufen und entsprechende Aktionen auslösen
void checkIgnitionState();
//Start bzw. Aufwecken
void startOdroid();
//Sofortiges geordnetes Herunterfahren
void stopOdroid();
//Odroid in Sleep versetzen und Display ausschalten
void pauseOdroid();
//Ein- und Ausgänge prüfen
void checkPins();
//Zeitstempel bauen
void buildtimeStamp();
//Mausrad simulieren, je nachdem in welche Richtung der iDrive Knopf gedreht wurde.
void scrollScreen();
//Taste drücken und sofort wieder freigeben
void sendKey(uint8_t keycode);
//Interaktion mit serieller Konsole
void readConsole();
//Lüftersteuerung
void fanControl();
//Display füllen
void feedDisplay();


/*------------- PIN DEFINITIONS -------------*/
//Opto 2 - Zündung Aktiv
const int PIN_IGNITION_INPUT = 6;
//Eingang Odroid Power Button. Wenn dieser HIGH ist, ist der Odroid aktiv.
const int PIN_ODROID_POWER_INPUT = 7;
//Ausgang Odroid Power Button
const int PIN_ODROID_POWER_BUTTON = A1;
//Power Button vom Display
const int PIN_ODROID_DISPLAY_POWER_BUTTON = A7;
//CAN CS Pin
const int SPI_CS_PIN = 10;
//Pin für Display Helligkeit
const int PIN_VU7A_BRIGHTNESS = A3;
//PWM Pin #1
const int PWM_PIN_1 = 2;
//PWM Pin #2
const int PWM_PIN_2 = 3;
//Pin für Steuerung des Lüfter Relais via OK
const int PIN_FAN_RELAY = A6;
//Status LED
const int STATUS_LED = A2;
//Aktivitäts-LED
const int ACT_LED = 5;
//Temperatursensor
const int TEMP_SENSOR = A0;
//CAN Interrupt
//const int CAN_INT = A1; //A1 = INT2

/*------------- Fields / Vars -------------*/


//Zeitstempel für Sekundentimer
unsigned long previousOneSecondTick = 0;

//Mögliche Aktionen
enum PendingAction
{
  ODROID_START,
  ODROID_STOP,
  ODROID_STANDBY,
  NONE
};
//Beinhaltet die aktuelle Aktion, welche ausgeführt wird oder werden soll.
//Sofern diese nicht NONE ist, können keine weiteren Aktionen ausgeführt werden.
//Dies soll doppelte Ausführungen von Start & Stop während der Hoch- und Herunterfahrphase des PCs verhindern
PendingAction pendingAction = NONE;

//Hier wird gespeichert, ob nach dem Ausführen einer Aktion eine weitere folgt
//Beispiel: Das Auto wird aufgesperrt und innerhalb des Startup-Intervalls wieder zugesperrt. Der PC würde nun eingeschaltet bleiben.
//Hier würde nun gespeichert werden, dass der PC wieder heruntergefahren werden soll, sobald der Timer abgelaufen ist.
PendingAction queuedAction = NONE;

//2 Sekunden für Aufwecken
const int ODROID_BOOT_HOLD_DELAY = 2100;
//5 Sekunden zum Herunterfahren (Lineage braucht nur 1 Sekunde bei aktivierung der Option "Shutdown without prompt")
const int ODROID_SHUTDOWN_HOLD_DELAY = 1000;

int odroidRunning = LOW; //Ergebnis vom Odroid GPIO #1. LOW = aus, HIGH = an

int lastIgnitionState = HIGH; //Hält den letzten Zündungsstatus

const int CYCLE_DELAY = 500; //Verzögerung in ms pro Schleifendurchlauf

int canModuleStatus = 1; //Zustand des CAN Moduls

unsigned long previousOdroidActionTime = 0;  //Vorherige Zeitmessung für Odroid Steuerung
unsigned long previousMainTaskTime = 0;      //Vorherige Zeitmessung für allgemeinen Timer
unsigned long previousIgnitionCheckTime = 0; //Vorherige Zeitmessung für Zündungsstatus
unsigned long previousOdroidPauseTime = 0;   //Vorherige Zeitmessung für Odroid Sleepmodus

bool odroidStartRequested = false;    //Start von Odroid angefordert
bool odroidShutdownRequested = false; //Stop von Odroid angefordert
bool odroidPauseRequested = false;    //Sleep oder Wakeup angefordert

bool startup = true; //Steuerung ist gerade angelaufen.

bool debugMode = false; //Debugmodus aktiv?

int ignitionOn = HIGH; //Zündung - HIGH = Aus, LOW = An

const int ODROID_STANDBY_HOLD_DELAY = 100;       //Button Press für Display und Sleep.
const unsigned long WAKEUP_WAIT_DELAY = 10000;   //10 Sekunden Wartezeit für Aufwecken
const unsigned long STARTUP_WAIT_DELAY = 60000;  //Wartezeit für Start
const unsigned long SHUTDOWN_WAIT_DELAY = 60000; //Wartezeit für Herunterfahren
unsigned long startPowerTransitionMillis = 0;    //Counter für den Aufweck- und Herunterfahrprozess
const unsigned long ODROID_STANDBY_DELAY = 5000; //Wartzeit für Sleepfunktion

//Geschwindigkeit der Seriellen Schnittstelle "Serial"
const int serialBaud = 115200;

//zuletzt errechneter Helligkeitswert für Display.
int lastBrightness = 0;

//Timer für RTC
unsigned long previousRtcUpdateMillis = 0UL;
unsigned long rtcAndWifiUpdateInterval = 60000UL;

//Langer Zeitstempel
char timeStamp[20] = "00:00:00 00.00.0000";

//Uhrzeit als Text
char timeString[9] = "00:00:00";

//Datum als Text
char dateString[11] = "00.00.0000";

//------ LEDs

//Startwert für Status LED
int statusLedBrightness = 0;

//Startwert für Aktivitäts-LED
int actLedBrightness = 0;

//Schrittweite für Helligkeitsveränderung
int statusLedStep = 10;
int actLedStep = 5;

//Intervall bzw. Geschwindigkeit der LED Zyklen in ms
unsigned long statusLedInterval = 10UL;
unsigned long actLedInterval = 10UL;

//Timer für LEDs

//Status LED
unsigned long statusLedMillis = 0L;

//ACT LED
unsigned long actLedMillis = 0L;

//Manual Flags
bool statusLedManual = false;
bool actLedManual = false;

//Fans
int dynamicDuty = MAX_FANDUTY;

//-----------------------------

//Displayhelligkeit

//Geringster Helligkeitswert vom Lichtsensor (Dunkelheit / abgedeckt)
const int MIN_LM_LIGHT_LEVEL = 17;
//Höchster Helligkeitswert vom Lichtsensor (Direktes Sonnenlicht)
const int MAX_LM_LIGHT_LEVEL = 80;
//Minimaler Steuerwert für Displayhelligkeit
const int MIN_DISPLAY_BRIGHTNESS = 50;
//Maximaler Steuerwert für Displayhelligkeit
const int MAX_DISPLAY_BRIGHTNESS = 255;

//---------- CAN

//CAN Modul verbinden
MCP_CAN CAN(SPI_CS_PIN);
//CAN Modul initialisieren
void setupCan();
//CAN Nachrichten prüfen und abarbeiten
void checkCan();

//CAN Nachrichten auf der Konsole ausgeben
void printCanMsg(int canId, unsigned char *buffer, int len);
//CAN Output als CSV
void printCanMsgCsv(int canId, unsigned char *buffer, int len);


//------------ BT


//BT Library
BPLib *BPMod;

//Initializes the bluetooth module
void setupBluetooth(Uart serial, int serialBaud);

//Initial Setup of a brand new module
void initalSetup();