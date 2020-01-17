/*
    Temperaturkontrolle des angeschlossenen PCs. Ein Temperatusensor sollte sich an dem Kühlkörper befinden und die Temperatur messen
*/

#pragma once
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

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

void setupTemperatureSensors()
{
    sensors.begin();
    sensors.setResolution(firstDeviceAddress, TEMP_RESOLUTION);
    sensors.setWaitForConversion(false);
    sensors.requestTemperatures();

    if (!sensors.getAddress(firstDeviceAddress, 0))
    {
        //Kein Sensor gefunden.
    }
    else
    {
        temperature = sensors.getTempCByIndex(0);
        sensors.setResolution(firstDeviceAddress, TEMP_RESOLUTION);
        sensors.requestTemperatures();
    }
}

float checkTemperature()
{
    sensors.setResolution(firstDeviceAddress, TEMP_RESOLUTION);
    sensors.requestTemperatures(); 
    //Filter
    temperature = temperature - 0.035 * (temperature - rawtemp) ;
    //Output
    return temperature;
}