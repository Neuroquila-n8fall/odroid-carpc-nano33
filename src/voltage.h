#pragma once
#include <Arduino.h>
#include <Adafruit_INA219.h>

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

void setupIna()
{
    // Initialize the INA219.
    // By default the initialization will use the largest range (32V, 2A).  However
    // you can call a setCalibration function to change this range (see comments).
    ina219.begin();
    // To use a slightly lower 32V, 1A range (higher precision on amps):
    //ina219.setCalibration_32V_1A();
    // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
    //ina219.setCalibration_16V_400mA();
}

void checkIna()
{
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);

    Serial.print("Bus Voltage:   ");
    Serial.print(busvoltage);
    Serial.println(" V");
    Serial.print("Shunt Voltage: ");
    Serial.print(shuntvoltage);
    Serial.println(" mV");
    Serial.print("Load Voltage:  ");
    Serial.print(loadvoltage);
    Serial.println(" V");
    Serial.print("Current:       ");
    Serial.print(current_mA);
    Serial.println(" mA");
    Serial.print("Power:         ");
    Serial.print(power_mW);
    Serial.println(" mW");
    Serial.println("");
}
