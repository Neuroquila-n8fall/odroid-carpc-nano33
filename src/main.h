#include <Arduino.h>
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
//CAN Nachrichten prüfen
void checkCan();
//Zeitstempel bauen
void buildtimeStamp();
//CAN Nachrichten auf der Konsole ausgeben
void printCanMsg(int canId, unsigned char *buffer, int len);
//CAN Output als CSV
void printCanMsgCsv(int canId, unsigned char *buffer, int len);
//Mausrad simulieren, je nachdem in welche Richtung der iDrive Knopf gedreht wurde.
void scrollScreen();
//Uhrzeit pflegen. Ist ausschließlich dazu da die Uhrzeit voran schreiten zu lassen, wenn der Canbus inaktiv ist und keine Zeit vom Auto kommt.
//Die RTC Library kommt leider nicht in Frage da mein DUE board wohl keinen Kristall für RTC hat und daher der MCU einfriert beim initialisieren.
void timeKeeper();
//Taste drücken und sofort wieder freigeben
void sendKey(uint8_t keycode);
//Interaktion mit serieller Konsole
void readConsole();
//PWM Setup
void setupPWM();

/*------------- PIN DEFINITIONS -------------*/
//Opto 2 - Zündung Aktiv
const int PIN_IGNITION_INPUT = 6;
//Eingang Odroid Power Button. Wenn dieser HIGH ist, ist der Odroid aktiv.
const int PIN_ODROID_POWER_INPUT = 7;
//Ausgang Odroid Power Button
const int PIN_ODROID_POWER_BUTTON = 8;
//Power Button vom Display
const int PIN_ODROID_DISPLAY_POWER_BUTTON = 9;
//CAN CS Pin
const int SPI_CS_PIN = 10;
//Pin für Display Helligkeit
const int PIN_VU7A_BRIGHTNESS = 5;
//Pin für Debug Switch
const int PIN_DEBUG = 53;
//PWM Pin #1
const int PWM_PIN_1 = 2;
//PWM Pin #2
const int PWM_PIN_2 = 3;
//Relay Pin für Lüfter OK
const int FAN_RELAY = A6;
//Status LED
const int STATUS_LED = A2;
//Aktivitäts-LED
const int ACT_LED = A3;
//Temperatursensor
const int TEMP_SENSOR = A0;
//CAN Interrupt
const int CAN_INT = A1; //A1 = INT2

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

//Stunden
int hours = 0;
//Minuten
int minutes = 0;
//Sekunden
int seconds = 0;
//Tag
int days = 0;
//Monat
int month = 0;
//Jahr
int year = 0;
//Langer Zeitstempel
char timeStamp[20] = "00:00:00 00.00.0000";
//Uhrzeit als Text
char timeString[9] = "00:00:00";
//Datum als Text
char dateString[11] = "00.00.0000";

//Initialstatus der eingebauten LED
int ledState = LOW;

//Displayhelligkeit

//Geringster Helligkeitswert vom Lichtsensor (Dunkelheit / abgedeckt)
const int MIN_LM_LIGHT_LEVEL = 17;
//Höchster Helligkeitswert vom Lichtsensor (Direktes Sonnenlicht)
const int MAX_LM_LIGHT_LEVEL = 80;
//Minimaler Steuerwert für Displayhelligkeit
const int MIN_DISPLAY_BRIGHTNESS = 50;
//Maximaler Steuerwert für Displayhelligkeit
const int MAX_DISPLAY_BRIGHTNESS = 255;


//PWM
//Minimaler Zyklus
const int MIN_FAN_DUTY = 0;
//Maximaler Zyklus
const int MAX_FAN_DUTY = 20000;
