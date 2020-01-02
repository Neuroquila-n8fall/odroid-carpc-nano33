#include <Arduino.h>

//Knopfdefinition

//Menü- oder Drehknopf gedrückt
const byte IDRIVE_BUTTON_CENTER_MENU = 0x01;
//Back
const byte IDRIVE_BUTTON_BACK = 0x02;
//Option
const byte IDRIVE_BUTTON_OPTION = 0x04;
//Radio
const byte IDRIVE_BUTTON_RADIO = 0x08;
//CD
const byte IDRIVE_BUTTON_CD = 0x10;
//NAV
const byte IDRIVE_BUTTON_NAV = 0x20;
//TEL
const byte IDRIVE_BUTTON_TEL = 0x40;
//Drehknopf wurde in eine Richtung bewegt
const byte IDRIVE_JOYSTICK = 0xDD;

//Joystick hoch
const byte IDRIVE_JOYSTICK_UP = 0x11;
//Joystick hoch gehalten
const byte IDRIVE_JOYSTICK_UP_HOLD = 0x12;
//Joystick rechts
const byte IDRIVE_JOYSTICK_RIGHT = 0x21;
//Joystick rechts gehalten
const byte IDRIVE_JOYSTICK_RIGHT_HOLD = 0x22;
//Joystick runter
const byte IDRIVE_JOYSTICK_DOWN = 0x41;
//Joystick runter gehalten
const byte IDRIVE_JOYSTICK_DOWN_HOLD = 0x42;
//Joystick links
const byte IDRIVE_JOYSTICK_LEFT = 0x81;
//Joystick links gehalten
const byte IDRIVE_JOYSTICK_LEFT_HOLD = 0x82;

//Adresse für Touch input
const int IDRIVE_CTRL_TOUCH_ADDR = 0x0BF;
//Touch-Modes
//Keine Finger
const int IDRIVE_CTRL_TOUCH_RELEASE = 0x11;
//Ein Finger
const int IDRIVE_CTRL_TOUCH_FINGER = 0x10;
//Zwei Finger
const int IDRIVE_CTRL_TOUCH_MULTI = 0x20;
