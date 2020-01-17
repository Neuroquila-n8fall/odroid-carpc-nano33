#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <arduino_secrets.h>
#include <ArduinoBLE.h>
#include <RTCZero.h>

//Sets up and connects wifi according to the constants defined in "arduino_secrets.h" as "SECRET_SSID" and "SECRET_PASS"
//This function throttles itself so it only tries to connect after a certain delay defined in "connect_delay"
void setupWifi();
//Setup UDP Connectivity
void setupUDP();
//Connects to a NTP server via UDP and retrieves the current time.
void updateTime();
//RTC Init
void setupRTC();
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

void setupWifi()
{
    if (previousConnectionAttempt == 0)
        previousConnectionAttempt = millis();
    if (millis() - previousConnectionAttempt >= connect_delay * 1000)
    {
        if (wifiStatus != WL_CONNECTED)
        {
            // Connect to WPA/WPA2 network:
            wifiStatus = WiFi.begin(SECRET_SSID, SECRET_PASS);
        }
    }
}

void setupUDP()
{
    Udp.begin(localPort);
}

void setupRTC()
{
    rtc.begin();
}

void updateTime()
{
    sendNTPpacket(timeServer); // send an NTP packet to a time server
    // wait to see if a reply is available
    delay(1000);
    if (Udp.parsePacket())
    {
        Serial.println("packet received");
        // We've received a packet, read the data from it
        Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

        //the timestamp starts at byte 40 of the received packet and is four bytes,
        // or two words, long. First, esxtract the two words:

        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
        unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
        // combine the four bytes (two words) into a long integer
        // this is NTP time (seconds since Jan 1 1900):
        unsigned long secsSince1900 = highWord << 16 | lowWord;

        // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
        const unsigned long seventyYears = 2208988800UL;
        // subtract seventy years:
        unsigned long epoch = secsSince1900 - seventyYears;

        //Update RTC, apply Berlin TZ = +3600 seconds
        rtc.setEpoch(epoch + 3600);
    }
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request

  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123

  Udp.write(packetBuffer, NTP_PACKET_SIZE);

  Udp.endPacket();

}
