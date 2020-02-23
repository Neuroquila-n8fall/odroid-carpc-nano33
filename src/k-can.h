//Timeout für CAN-Bus
const int CAN_TIMEOUT = 30000;

/*
  Konstanten für CAS
*/
//CAS Debounce Timeout
const int CAS_DEBOUNCE_TIMEOUT = 1000;

/*
  Konstanten für MFL Knöpfe
*/
//MFL Button HOLD Threshold: Nach wie vielen Wiederholungen wird der Knopf als gehalten betrachtet.
const int mflButtonHoldThreshold = 3;
//MFL HOLD Time: Innerhalb wie vieler ms mflButtonHoldThreshold erreicht werden muss.
const unsigned int mflButtonHoldTime = 1000;

/*
  Variablen für MFL Knöpfe
*/
//Debounce für MFL Knopfdruck
unsigned long lastMflPress = 0;
//Debounce Timestamp für MFL release
unsigned long lastMflRelease = 0;
//Ob der MFL Knopf "Next" gehalten wird
bool MflButtonNextPressed = false;
//Ob der MFL Knopf "Prev" gehalten wird
bool MflButtonPrevPressed = false;
//MFL Button Counter
int mflButtonCounter = 0;

/*
  Variablen für Canbus Nachrichten
*/
//Zeit seit dem letzten Empfangen einer Uhrzeitnachricht über CAN
unsigned long previousCanDateTime = 0;
//Zeitstempel der zuletzt empfangenen CAN-Nachricht
unsigned long previousCanMsgTimestamp = 0;
//Zeitstempel für letzte Canbus Nachricht
unsigned long previousCasMessageTimestamp = 0;
//FALSE, wenn der Canbus seit mehr als 30 sekunden keine Nachricht transportiert hat.
bool canbusEnabled = false;

/*
  iDrive Implementation
*/

/*  
  Konstanten für iDrive Implementation
*/

//Keepalive Intervall
const int IDRIVE_KEEPALIVE_INTERVAL = 500;
//Init Timeout
const int IDRIVE_INIT_TIMEOUT = 750;

//Timer für Keepalive
unsigned long previousIdrivePollTimestamp = 0;

//Init Timer
unsigned long previousIdriveInitTimestamp = 0;
//Initialisierung erfolgreich
bool iDriveInitSuccess = false;

//TRUE, wenn Initialwert für Drehknopf am iDrive bereits errechnet wurde
bool RotaryInitPositionSet = false;
//Zähler für Drehung des Drehknopfes
unsigned int rotaryposition = 0;

//iDrive Stuff
enum iDriveRotationDirection
{
  ROTATION_RIGHT,
  ROTATION_LEFT,
  ROTATION_NONE
};

//iDrive Knopf gedreht?
bool iDriveRotChanged = false;
//Zuletzt gesicherte Drehung des Knopfes um Richtung zu bestimmen
int iDriveRotLast = 0;
//Drehungs-counter
int iDriveRotCountLast = 0;
//Drehrichtung
iDriveRotationDirection iDriveRotDir = ROTATION_NONE;

//iDrive Button debounce limit
const int IDRIVE_BUTTON_DEBOUNCE = 500;
//iDrive Button debounce timestamp
unsigned long previousIdriveButtonTimestamp = 0;
//iDrive Joystick debounce limit
const int IDRIVE_JOYSTICK_DEBOUNCE = 500;
//iDrive Joystick debounce timestamp
unsigned long previousIdriveJoystickTimestamp = 0;
//Zuletzt gedrückte Taste
byte previousIdriveButton = 0x00;

enum iDriveButtonPressType
{
  BUTTON_SHORT_PRESS,
  BUTTON_LONG_PRESS,
  BUTTON_RELEASE
};

iDriveButtonPressType iDriveBtnPress = BUTTON_RELEASE;

//Zuletzt empfangener Counter der iDrive Funktionsknöpfe.
//Wird immer erhöht, wenn eine neue Aktion ausgeführt wird.
int previousIdriveButtonPressCounter = 0;

//Zuletzt bekannte Art des Tastendrucks
byte lastKnownIdriveButtonPressType = 0x00;
//Zuletzt bekannte Funktionstaste
byte lastKnownIdriveButtonType = 0x00;

//        iDrive ENDE