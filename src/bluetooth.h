#pragma once
#include <Arduino.h>
#include <BPLib.h>

//BT Library
BPLib *BPMod;

//Initializes the bluetooth module
void setupBluetooth(Uart serial, int serialBaud);

//Initial Setup of a brand new module
void initalSetup();

void setupBluetooth(Uart serial, int serialBaud)
{
  // Bluetooth Modul
  //Serial1 (0/1 für BT Modul benutzen)
  Serial.println("[setup] Serial 1 (Pin RX0,TX1)");
  serial.begin(serialBaud);
  Serial.println("[setup] Setup BT Library");
  BPMod = new BPLib(serial);
}

void initalSetup()
{
    //Command Reference: https://cdn.sparkfun.com/datasheets/Wireless/Bluetooth/bluetooth_cr_UG-v1.0r.pdf
    //Set as a Combo device
    BPMod->sendCmd("SH,0230");
    //Enable HID Profile
    BPMod->sendCmd("S~,6");
    //Set PIN Mode
    BPMod->sendCmd("SA,4");
    //Set PIN
    BPMod->sendCmd("SP,1234");
    //Set device name
    BPMod->sendCmd("SM,IDRIVECTRL");


}