#include <Arduino.h>
#include <SPI.h>

/*
  Projektbezogene Header
*/

#include <main.h>
#include <fan_pwm.h>
#include <voltage.h>
#include <connectivity.h>
#include <oled.h>
#include <bluetooth.h>
#include <k-can-processor.h>
#include <temperature.h>

void setup()
{
  setupOled();
  SeeedOled.setTextXY(0, 0);
  SeeedOled.putString("Booting...");

  analogWrite(STATUS_LED, 255);
  digitalWrite(ACT_LED, 0);
  Serial.begin(serialBaud);

  //Zeitstempel einrichten
  buildtimeStamp();

  pinMode(PIN_IGNITION_INPUT, INPUT_PULLUP);        //Zündungs-Pin
  pinMode(PIN_ODROID_POWER_BUTTON, OUTPUT);         //Opto 2 - Odroid Power Button
  pinMode(PIN_ODROID_POWER_INPUT, INPUT_PULLUP);    //Odroid VOUT Pin als Rückmeldung ob der PC eingeschaltet ist
  pinMode(PIN_ODROID_DISPLAY_POWER_BUTTON, OUTPUT); //Opto 3 - Display Power Button
  pinMode(STATUS_LED, OUTPUT);                      //Status LED
  pinMode(ACT_LED, OUTPUT);                         //Activity LED
  pinMode(PIN_DEBUG, INPUT_PULLUP);                 //Debug Switch Pin
  pinMode(PIN_VU7A_BRIGHTNESS, OUTPUT);             //Display Helligkeitssteuerung
  pinMode(PIN_FAN_RELAY, OUTPUT);                   //Lüfter Relais

  setupCan();
  setupIna();
  setupBluetooth(Serial1, serialBaud);
  setupPWM();
  //Lüfter einschalten
  digitalWrite(PIN_FAN_RELAY, HIGH);
  setupWifi();
  setupUDP();
  setupRTC();
  setupTemperatureSensors();

  //Display von ganz dunkel nach ganz Hell stellen. Quasi als Test
  for (int i = 0; i <= 255; i++)
  {
    analogWrite(PIN_VU7A_BRIGHTNESS, i);
  }
  analogWrite(STATUS_LED, 255);
  Serial.println("[setup] Ready.");
  SeeedOled.setTextXY(0, 0);
  SeeedOled.putString("Ready     ");
}

void loop()
{
  //Aktuelle Zeit
  unsigned long currentMillis = millis();

  if (startup)
  {
    //Wenn die Steuerung mit aktiver Zündung startet oder resettet, sollte der Odroid starten
    //Aber nur wenn der Odroid AUS ist aber die Zündung an
    //Wenn der Odroid im Sleep ist, ist odroidRunning ebenfalls Low. Auf diese Art wird er bei Zündung direkt aufgeweckt.
    if (odroidRunning == LOW && ignitionOn == LOW && odroidStartRequested == false)
    {
      Serial.println("[STARTUP] PC aus, Zündung an --> Starte Pc...");
      startOdroid();
    }
    //Start beendet.
    startup = false;
  }

  //Start angefordert
  if (odroidStartRequested)
  {
    if (currentMillis - previousOdroidActionTime >= ODROID_BOOT_HOLD_DELAY)
    {
      odroidStartRequested = false;
      //Zeit merken
      previousOdroidActionTime = currentMillis;
      //Ausgang freigeben
      digitalWrite(PIN_ODROID_POWER_BUTTON, LOW);
      Serial.println("[odroidStartRequested] Start erfolgt.");
      //Aktuellen Counter merken. Ausführung weiterer Aktionen wird pausiert, bis abgewartet wurde.
      startPowerTransitionMillis = millis();
    }
  }
  //Stop angefordert
  if (odroidShutdownRequested)
  {
    if (currentMillis - previousOdroidActionTime >= ODROID_SHUTDOWN_HOLD_DELAY)
    {
      odroidShutdownRequested = false;
      //Zeit merken
      previousOdroidActionTime = currentMillis;
      //Ausgang freigeben
      digitalWrite(PIN_ODROID_POWER_BUTTON, LOW);
      Serial.println("[odroidShutdownRequested] Stop erfolgt.");
      //Aktuellen Counter merken. Ausführung weiterer Aktionen wird pausiert, bis abgewartet wurde.
      startPowerTransitionMillis = millis();
    }
  }

  if (odroidPauseRequested)
  {
    if (currentMillis - previousOdroidPauseTime >= ODROID_STANDBY_DELAY)
    {
      odroidPauseRequested = false;
      Serial.println("[odroidPauseRequested] Standby erfolgt.");
      //Aktuellen Counter merken. Ausführung weiterer Aktionen wird pausiert, bis abgewartet wurde.
      startPowerTransitionMillis = millis();
    }
  }

  //CAN Nachrichten verarbeiten
  checkCan();
  //Konsole
  readConsole();

  //1-Second timer
  if (currentMillis - previousOneSecondTick >= 1000)
  {
    previousOneSecondTick = currentMillis;
    //Zeitstempel Variablen füllen
    buildtimeStamp();
    //Lüfter steuern
    fanControl();
    //Spannung abrufen
    checkIna();
    //Display
    feedDisplay();
  }

  //Status-LED
  if (currentMillis - statusLedMillis >= statusLedInterval)
  {
    //Wenn CAN Aktiv
    if (canbusEnabled)
    {
      //Wenn PC läuft, Dauerleuchten.
      if (odroidRunning == HIGH)
      {
        statusLedInterval = 1000;
        analogWrite(STATUS_LED, 255);
        statusLedBrightness = 255;
      }
      else
      {
        //Wenn der PC nicht läuft, blinken
        statusLedInterval = 500;
        if (statusLedBrightness != 0 || statusLedBrightness != 255)
        {
          statusLedBrightness = 255;
        }
        analogWrite(STATUS_LED, statusLedBrightness);
        //Invertieren
        statusLedBrightness -= statusLedBrightness;
      }
    }
    else
    {
      //Wenn der CAN inaktiv ist, pulsieren um sowas wie "Stand-By" zu zeigen.
      statusLedInterval = 100;
      statusLedBrightness -= statusLedStep;
      //Richtung umkehren bei MIN oder MAX Wert
      if (statusLedBrightness <= 0 || statusLedBrightness >= 255)
      {
        statusLedStep -= statusLedStep;
      }
    }
  }

  bool anyPendingActions = odroidStartRequested || odroidShutdownRequested || odroidPauseRequested;

  //Aktivitäts-LED
  if (currentMillis - actLedMillis >= actLedInterval)
  {
    switch (pendingAction)
    {
    case ODROID_STOP:
      actLedInterval = 100;

      break;
    case ODROID_START:
      actLedInterval = 10;
      break;

    default:
      break;
    }
    actLedBrightness -= actLedStep;
    //Richtung umkehren bei MIN oder MAX Wert
    if (actLedBrightness <= 0 || actLedBrightness >= 255)
    {
      actLedStep -= actLedStep;
    }
  }

  //Allgemeine Funktionen. Nur ausführen, wenn Zyklus erreicht wurde und keine ausstehenden Aktionen laufen, die ein zeitkritisches Verändern der Ausgänge beinhalten.
  if (currentMillis - previousMainTaskTime >= CYCLE_DELAY && !anyPendingActions)
  {
    //Zündung überprüfen
    checkIgnitionState();
    //Ein- und Ausgänge überprüfen
    checkPins();

    //Zeit merken
    previousMainTaskTime = currentMillis;
    //Sperrung freigeben, wenn Timeout abgelaufen ist
    switch (pendingAction)
    {
    case ODROID_STOP:
      if (currentMillis - startPowerTransitionMillis >= SHUTDOWN_WAIT_DELAY)
      {
        Serial.println("[LOOP] Shutdown Wartezeit abgelaufen");
        pendingAction = NONE;
        //Wurde der Start vorgemerkt, ausführen und zurücksetzen
        if (queuedAction == ODROID_START)
        {
          startOdroid();
          queuedAction = NONE;
        }
      }
      break;
    case ODROID_START:
      if (currentMillis - startPowerTransitionMillis >= STARTUP_WAIT_DELAY)
      {
        Serial.println("[LOOP] Start Wartezeit abgelaufen");
        pendingAction = NONE;
        //Wurde Stopp vorgemerkt, ausführen und zurücksetzen
        if (queuedAction == ODROID_STOP)
        {
          stopOdroid();
          queuedAction = NONE;
        }
      }
      break;
    case ODROID_STANDBY:
      if (currentMillis - startPowerTransitionMillis >= ODROID_STANDBY_DELAY)
      {
        Serial.println("[LOOP] Stand-by Wartezeit abgelaufen");
        pendingAction = NONE;
      }
      break;

    default:
      //Keine Aktion aktiv.

      //Sicherheitshalber zurücksetzen
      queuedAction = NONE;
      break;
    }
  }
}

void checkPins()
{
  //Status des Zündungspins abrufen
  ignitionOn = digitalRead(PIN_IGNITION_INPUT);
  //Staus Odroid Vcc pin
  odroidRunning = !digitalRead(PIN_ODROID_POWER_INPUT);

  //Status Debug-Pin
  debugMode = !digitalRead(PIN_DEBUG);

  //Prüfe alle Faktoren für Start, Stopp oder Pause des Odroid.
  checkIgnitionState();
}

void checkIgnitionState()
{

  //Wenn der Status der Zündung sich verändert hat.
  if (ignitionOn != lastIgnitionState)
  {
    //Zündung ist an aber PC ist aus.
    if (ignitionOn == LOW && odroidRunning == LOW)
    {
      //PC starten.
      startOdroid();
    }
  }
  //Letzten Status merken.
  lastIgnitionState = ignitionOn;
}

void startOdroid()
{
  //Zeitstempel
  Serial.print(timeStamp + '\t');
  Serial.print("[startOdroid] Odroid Status:");
  Serial.println(odroidRunning == LOW ? "AUS" : "AN");
  //Mehrfachen Aufruf verhindern - auch wenn der PC bereits läuft
  if (odroidStartRequested || odroidRunning || pendingAction != NONE)
  {
    return;
  }

  //Starten
  odroidStartRequested = true;
  pendingAction = ODROID_START;
  digitalWrite(PIN_ODROID_POWER_BUTTON, HIGH);
  //Zeitstempel
  Serial.print(timeStamp + '\t');
  Serial.println("[startOdroid] Start angefordert.");
  previousOdroidActionTime = millis();
}

void pauseOdroid()
{
  //Wenn der PC aus ist, dann brauchen wir auch nicht Pause drücken.... Wir gehen auch einfach mal davon aus, dass er aus ist.
  //Wenn auch noch eine Stand-By Anforderung ausstehend ist, könnte ein erneutes Drücken den Start wieder auslösen.
  if (odroidPauseRequested || !odroidRunning || pendingAction != NONE)
  {
    return;
  }
  pendingAction = ODROID_STANDBY;
  digitalWrite(PIN_ODROID_DISPLAY_POWER_BUTTON, HIGH);
  digitalWrite(PIN_ODROID_POWER_BUTTON, HIGH);
  Serial.println("[pauseOdroid] Stand-By angefordert");
  //Kurze Verzögerung - kurzer Tastendruck für display und Odroid
  delay(ODROID_STANDBY_HOLD_DELAY);
  digitalWrite(PIN_ODROID_DISPLAY_POWER_BUTTON, LOW);
  digitalWrite(PIN_ODROID_POWER_BUTTON, LOW);
  odroidPauseRequested = true;
  previousOdroidPauseTime = millis();
}

void stopOdroid()
{
  //Zeitstempel
  Serial.print(timeStamp + '\t');
  Serial.print("[stopOdroid] Odroid Status:");
  Serial.println(odroidRunning == LOW ? "AUS" : "AN");
  //Mehrfachen Aufruf verhindern - auch wenn der PC bereits aus ist. Das würde diesen nämlich einschalten.
  if (odroidShutdownRequested || !odroidRunning || pendingAction != NONE)
  {
    return;
  }

  //Herunterfahren
  odroidShutdownRequested = true;
  pendingAction = ODROID_STOP;

  digitalWrite(PIN_ODROID_POWER_BUTTON, HIGH);
  Serial.println("[stopOdroid] Herunterfahren angefordert");

  previousOdroidActionTime = millis();
}

void buildtimeStamp()
{
  sprintf(timeStamp, "[%02d:%02d:%02d %02d.%02d.%u]", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds(), rtc.getDay(), rtc.getMonth(), rtc.getYear());
  sprintf(timeString, "%02d:%02d:%02d", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  sprintf(dateString, "%02d.%02d.%u", rtc.getDay(), rtc.getMonth(), rtc.getYear());
}

void readConsole()
{
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    if (command == "pc.stop")
    {
      stopOdroid();
    }
    if (command == "pc.start")
    {
      startOdroid();
    }
    if (command == "pc.pause")
    {
      pauseOdroid();
    }
    if (command == "car.testcan")
    {
    }
  }
}

void fanControl()
{
  //Lüfter einschalten, wenn PC läuft. Ansonsten ausschalten.
  if (odroidRunning = HIGH)
  {
    digitalWrite(PIN_FAN_RELAY, HIGH);
  }
  else
  {
    digitalWrite(PIN_FAN_RELAY, LOW);
  }

  //Temperaturabhängige Lüftersteuerung
  temperature = checkTemperature();

  if (temperature < 20)
  {
    setFanDutyCycle(MIN_FANDUTY);
  }
  if (temperature > 40)
  {
    setFanDutyCycle(MAX_FANDUTY);
  }

  if (temperature >= 20 && temperature <= 40)
  {
    int dynamicDuty = map((int)temperature, 20, 40, 0, 1919);
    setFanDutyCycle(dynamicDuty);
  }
}

void feedDisplay()
{
  int row = 0;
  //---------- 1
  SeeedOled.setTextXY(row, 0);
  SeeedOled.putString("ACT:");
  SeeedOled.setTextXY(row, 6);
  switch (pendingAction)
  {
  case ODROID_START:
    SeeedOled.putString("Start");
    break;
  case ODROID_STOP:
    SeeedOled.putString("Stop");
    break;
  case NONE:
    SeeedOled.putString("Idle");
    break;
  default:
    break;
  }
  //---------- 2
  SeeedOled.setTextXY(row++, 0);
  SeeedOled.putString("TC:");
  SeeedOled.setTextXY(row, 6);
  char tempFt[7] = "999°C";
  sprintf(tempFt,"%03d°C");
  //---------- 3
  SeeedOled.setTextXY(row++, 0);
  SeeedOled.putString("V:");
  SeeedOled.setTextXY(row, 6);
  SeeedOled.putFloat(loadvoltage);
}
