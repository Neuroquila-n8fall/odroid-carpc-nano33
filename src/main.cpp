#include <Arduino.h>

/*
  Projektbezogene Header
*/

#include <main.h>
#include <k-can.h>
#include <k-can-messages.h>
#include <k-can-addresses.h>
#include <idrive-controls.h>

#pragma region "Setup"
void setup()
{
    
    pinMode(PIN_IGNITION_INPUT, INPUT_PULLUP);        //Zündungs-Pin
    pinMode(PIN_ODROID_POWER_BUTTON, OUTPUT);         //Opto 2 - Odroid Power Button
    pinMode(PIN_ODROID_POWER_INPUT, INPUT_PULLUP);    //Odroid VOUT Pin als Rückmeldung ob der PC eingeschaltet ist
    pinMode(PIN_ODROID_DISPLAY_POWER_BUTTON, OUTPUT); //Opto 3 - Display Power Button
    pinMode(STATUS_LED, OUTPUT);                      //Status LED
    pinMode(ACT_LED, OUTPUT);                         //Activity LED
    pinMode(PIN_DEBUG, INPUT_PULLUP);                 //Debug Switch Pin
    pinMode(PIN_VU7A_BRIGHTNESS, OUTPUT);             //Display Helligkeitssteuerung
    pinMode(PIN_FAN_RELAY, OUTPUT);                   //Lüfter Relais
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(A7, OUTPUT);

    analogWrite(A7, 255);
    
    Serial.println("[setup] Startup");

    Serial.begin(serialBaud);

    //Zeitstempel einrichten
    buildtimeStamp();

    
    setupIna();
    setupBluetooth(Serial1, serialBaud);
    //First-Time use only
    //initalSetup();

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
    setupCan();
    setupPWM();
}
#pragma endregion

#pragma region "LOOP"
void loop()
{
    //Aktuelle Zeit
    unsigned long currentMillis = millis();
    bool anyPendingActions = odroidStartRequested || odroidShutdownRequested || odroidPauseRequested;

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
        //feedDisplay();
        Serial.println("---------- Update ----------");
        Serial.print("Uhrzeit: ");
        Serial.println(timeString);
        Serial.print("Datum: ");
        Serial.println(dateString);
        Serial.print("Temperatur: ");
        Serial.print(temperature);
        Serial.println("°C");
        Serial.print("Lüftergeschwindigkeit: ");
        Serial.print(map(dynamicDuty,0,MAX_FANDUTY,0,100));
        Serial.println("%");
        Serial.print("Spannung: ");
        Serial.print(busvoltage);
        Serial.println("V");
        Serial.print("Zündung: ");
        Serial.println(!ignitionOn);
        Serial.print("PC: ");
        Serial.println(odroidRunning);
        Serial.print("Action: ");
        switch (pendingAction)
        {
        case ODROID_START:
            Serial.println("Starten");
            break;
        case ODROID_STOP:
            Serial.println("Stoppen");
            break;
        case NONE:
            Serial.println("Idle.");
            break;
        default:
            break;
        }
    }

    //Zeit alle 60 Sekunden aktualisieren bzw. Wifi Verbindung aufbauen. Da das blockieren kann, wird das nur ausgeführt, wenn keine Aktion gerade ansteht
    if (currentMillis - previousRtcUpdateMillis >= 60000 && !anyPendingActions)
    {

        previousRtcUpdateMillis = currentMillis;
        Serial.println("[WiFi] Verbindung checken");
        setupWifi();
        Serial.println("[RTC] Zeit aktualisieren");
        updateTime();
    }

    //Status-LED
    if (currentMillis - statusLedMillis >= statusLedInterval)
    {
        statusLedMillis = currentMillis;
        if (odroidRunning == HIGH)
        {
            statusLedBrightness = 255;
        }
        else
        {
            if (canbusEnabled)
            {
                statusLedInterval = 100UL;
            }
            else
            {
                statusLedInterval = 500UL;
            }
            statusLedBrightness += statusLedStep;
            if (statusLedBrightness <= 0 || statusLedBrightness >= 255)
            {
                statusLedStep = -statusLedStep;
            }
        }

        analogWrite(STATUS_LED, statusLedBrightness);
    }

    //Aktivitäts-LED
    if (currentMillis - actLedMillis >= actLedInterval)
    {
        actLedMillis = currentMillis;
        switch (pendingAction)
        {
        case ODROID_STOP:
            actLedInterval = 100UL;

            break;
        case ODROID_START:
            actLedInterval = 10UL;
            break;

        default:
            break;
        }
        actLedBrightness += actLedStep;
        //Richtung umkehren bei MIN oder MAX Wert
        if (actLedBrightness <= 0 || actLedBrightness >= 255)
        {
            actLedStep = -actLedStep;
        }
        analogWrite(ACT_LED,actLedBrightness);
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
#pragma endregion

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
        if (command == "debug.activate")
        {
            debugMode = true;
        }
        if (command == "debug.deactivate")
        {
            debugMode = false;
        }
    }
}

void fanControl()
{
    //Lüfter einschalten, wenn PC läuft. Ansonsten ausschalten.
    if (odroidRunning == HIGH)
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
        dynamicDuty = MIN_FANDUTY;
        digitalWrite(PIN_FAN_RELAY, LOW);
    }
    if (temperature > 40)
    {
        dynamicDuty = MAX_FANDUTY;
    }

    if (temperature >= 20 && temperature <= 40)
    {
        dynamicDuty = map((int)temperature, 20, 40, MIN_FANDUTY, MAX_FANDUTY);
        
    }
    setFanDutyCycle(dynamicDuty);
}

#pragma region "Bluetooth"

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
    Serial1.write("$$$");
    //Wait for device response
    delay(250);
    //Set as a Combo device
    Serial1.write("SH,0230");
    //Enable HID Profile
    Serial1.write("S~,6");
    //Set PIN Mode
    Serial1.write("SA,4");
    //Set PIN
    Serial1.write("SP,1234");
    //Set device name
    Serial1.write("SN,IDRIVECTRL");
    //End
    Serial1.write("---");
}
#pragma endregion

#pragma region "CAN-Processor"
void setupCan()
{
    if (canModuleStatus != CAN_OK)
    {
        Serial.println("[CAN MODUL] Nicht initialisiert. Initialisiere...");
        delay(5000);
        canModuleStatus = CAN.begin(CAN_100KBPS);
        if (canModuleStatus != CAN_OK)
        {
            Serial.println("[CAN MODUL] Initialisierung fehlgeschlagen.");
            delay(5000);
        }
    }
    else
    {
        //nix
    }
}

void checkCan()
{
    //Abbruch, wenn Modul nicht bereit
    if (canModuleStatus != CAN_OK)
    {
        return;
    }

    unsigned char len = 0;
    unsigned char buf[8];
    unsigned long currentMillis = millis();

    if (CAN_MSGAVAIL == CAN.checkReceive())
    {
        previousCanMsgTimestamp = currentMillis;
        canbusEnabled = true;
        CAN.readMsgBuf(&len, buf);

        unsigned int canId = CAN.getCanId();

        //Aktivitäts-LED ansteuern
        if (pendingAction == NONE)
        {
            if (actLedBrightness == 0)
            {
                actLedBrightness = 255;
            }
            else
            {
                actLedBrightness = 0;
            }            
            analogWrite(ACT_LED, actLedBrightness);
        }
        //Alle CAN Nachrichten ausgeben, wenn debug aktiv.
        if (debugMode)
        {
            printCanMsgCsv(canId, buf, len);
        }

        switch (canId)
        {
        //MFL Knöpfe
        case MFL_BUTTON_ADDR:
        {
            //Kein Knopf gedrückt (alle 1000ms)
            if (buf[0] == 0xC0 && buf[1] == 0x0C)
            {
                //Taste wurde losgelassen und zuvor nicht lang genug gedrückt gehalten.
                if (mflButtonCounter < mflButtonHoldThreshold)
                {
                    //NEXT oder PREV
                    if (MflButtonNextPressed)
                    {
                        Serial.println("[MFL] NEXT");
                        BPMod->nextTrack();
                        BPMod->keyRelease();
                    }
                    if (MflButtonPrevPressed)
                    {
                        Serial.println("[MFL] PREV");
                        BPMod->prevTrack();
                        BPMod->keyRelease();
                    }
                }
                //Knöpfe zurücksetzen
                MflButtonNextPressed = false;
                MflButtonPrevPressed = false;
                //Zeitstempel merken
                lastMflRelease = currentMillis;
                //Counter zurücksetzen
                mflButtonCounter = 0;
            }

            //Wenn innerhalb einer Zeitspanne das selbe Tastensignal gesendet wird, wird der Knopf als gehalten betrachtet, aber nur wenn der Schwellwert erreicht wurde
            //Erneuter Tastendruck innerhalb von Zeit
            if (currentMillis - lastMflPress <= 200)
            {
                mflButtonCounter++;
                //Wenn der Knopf seit bestimmter Zeit nicht gelöst wurde und der Schwellwert erreicht wurde
                if (currentMillis - lastMflRelease >= mflButtonHoldTime && mflButtonCounter > mflButtonHoldThreshold)
                {
                    //FASTFORWARD oder REWIND
                    //Next
                    if (buf[0] == 0xE0 && buf[1] == 0x0C)
                    {
                        Serial.println("[MFL] FASTFORWARD");
                        BPMod->fastForwardAudio();
                        BPMod->keyRelease();
                    }
                    //Prev
                    if (buf[0] == 0xD0 && buf[1] == 0x0C)
                    {
                        Serial.println("[MFL] REWIND");
                        BPMod->rewindAudio();
                        BPMod->keyRelease();
                    }
                }
            }
            //Next
            if (buf[0] == 0xE0 && buf[1] == 0x0C)
            {
                MflButtonNextPressed = true;
            }
            //Prev
            if (buf[0] == 0xD0 && buf[1] == 0x0C)
            {
                MflButtonPrevPressed = true;
            }

            lastMflPress = currentMillis;

            //Pickup Button
            if (buf[0] == 0xC1 && buf[1] == 0x0C)
            {
            }
            //Voice Button
            if (buf[0] == 0xC0 && buf[1] == 0x0D)
            {
            }
            break;
        }
        //CAS: Schlüssel & Zündung
        case CAS_ADDR:
        {
            //Wakeup Signal vom CAS --> Alle Steuergeräte aufwecken
            //Wird alle 100ms geschickt
            if (buf[0] == 0x45)
            {
            }
            //Wenn der Schlüssel im Fach ist, ist der Wert größer 0x0
            if (buf[0] > 0)
            {
                if (!iDriveInitSuccess)
                {
                    CAN.sendMsgBuf(IDRIVE_CTRL_INIT_ADDR, 0, 8, IDRIVE_CTRL_INIT);
                }
            }
            break;
        }
        //CIC
        case CIC_ADDR:
        {
            Serial.print("CIC\t");
            printCanMsg(canId, buf, len);
            break;
        }
        case IDRIVE_CTRL_INIT_RESPONSE_ADDR:
        {
            iDriveInitSuccess = true;
            break;
        }
        case IDRIVE_CTRL_STATUS_ADDR:
        {
            if (buf[4] == 6)
            {
                Serial.println("Controller ist nicht initialisiert.");
                //Controller meldet er sei nicht initialisiert: Nachricht zum Initialisieren senden.
                CAN.sendMsgBuf(IDRIVE_CTRL_INIT_ADDR, 0, 8, IDRIVE_CTRL_INIT);
                iDriveInitSuccess = false;
            }
            else
            {
                iDriveInitSuccess = true;
            }
            break;
        }
        //CAS: Schlüssel Buttons
        case 0x23A:
        {
            //Debounce: Befehle werden erst wieder verarbeitet, wenn der Timeout abgelaufen ist.
            if (currentMillis - previousCasMessageTimestamp > CAS_DEBOUNCE_TIMEOUT)
            {
                previousCasMessageTimestamp = currentMillis;
                //Öffnen:     00CF01FF
                if (buf[0] == 0x00 && buf[1] == 0x30 && buf[2] == 0x01 && buf[3] == 0x60)
                {
                    //Prüfen, ob der PC noch im Begriff ist herunter zu fahren
                    if (pendingAction == ODROID_STOP)
                    {
                        queuedAction = ODROID_START;
                        Serial.println("[checkCan] PC wird nach dem Herunterfahren wieder gestartet.");
                    }
                    //Starten
                    startOdroid();

                    //Controller initialisieren.
                    CAN.sendMsgBuf(IDRIVE_CTRL_INIT_ADDR, 0, 8, IDRIVE_CTRL_INIT);
                    previousIdriveInitTimestamp = currentMillis;
                    //Zur Kontrolle die Instrumentenbeleuchtung einschalten.
                    CAN.sendMsgBuf(DASHBOARD_LIGHTING_ADDR, 0, 8, DASHBOARD_LIGHTING_ON);
                    //Displayhelligkeit auf Maximum
                    analogWrite(PIN_VU7A_BRIGHTNESS, 255);
                }
                //Schließen:  00DF40FF
                if (buf[0] == 0x00 && buf[1] == 0x30 && buf[2] == 0x04 && buf[3] == 0x60)
                {
                    //Prüfen, ob der PC noch im Begriff ist hochzufahren
                    if (pendingAction == ODROID_START)
                    {
                        queuedAction = ODROID_STOP;
                        Serial.println("[checkCan] PC wird nach dem Starten wieder heruntergefahren.");
                    }
                    stopOdroid();
                    //Displayhelligkeit auf vertretbares Minimum
                    analogWrite(PIN_VU7A_BRIGHTNESS, 50);
                }
                //Kofferraum: Wird nur gesendet bei langem Druck auf die Taste
            }
            break;
        }
        //RLS Lichtsensorik
        case RLS_ADDR:
        {
            //Pralle Sonne:
            //[314]   50      0       FF
            //Tuch drüber gelegt:
            //[314]   11      8       FF

            //unsigned int lightVal = (buf[1] << 8) + buf[0];
            //Licht ist definitiv auf byte 0 aber keine Ahnung ob byte 1 noch was zu sagen hat. Dieses byte wechselt jedenfalls nach 8, wenn dann auch das Abblendlicht angeht.

            int lightValue = buf[0];

            //Lichtwert prüfen und bei Über- oder Unterschreitungen korrigieren. Die MAP Funktion kippt sonst nämlich die Werte, was zu einem schwarzen Bildschirm führen kann.
            if (lightValue > MAX_LM_LIGHT_LEVEL)
            {
                lightValue = MAX_LM_LIGHT_LEVEL;
            }
            if (lightValue < MIN_LM_LIGHT_LEVEL)
            {
                lightValue = MIN_LM_LIGHT_LEVEL;
            }

            //Display auf volle Helligkeit einstellen. Das ist unser Basiswert
            int val = 255;

            val = map(lightValue, MIN_LM_LIGHT_LEVEL, MAX_LM_LIGHT_LEVEL, MIN_DISPLAY_BRIGHTNESS, MAX_DISPLAY_BRIGHTNESS);

            //Wenn der aktuelle Wert größer als der zuletzt gespeicherte ist, zählen wir vom letzten Wert hoch.
            if (val > lastBrightness)
            {
                for (int i = lastBrightness; i <= val; i++)
                {
                    analogWrite(PIN_VU7A_BRIGHTNESS, i);
                    delay(10);
                }
            }

            //Wenn der aktuelle Wert kleiner als der zuletzt gespeicherte ist, dann schrittweise die Helligkeit vom letzten bekannten Wert absenken
            if (val < lastBrightness)
            {
                for (int i = lastBrightness; i >= val; i--)
                {
                    analogWrite(PIN_VU7A_BRIGHTNESS, i);
                    delay(10);
                }
            }

            //Wenn der Wert unverändert ist zur Sicherheit nochmals letzten Wert schreiben.
            if (val == lastBrightness)
            {
                analogWrite(PIN_VU7A_BRIGHTNESS, val);
                break;
            }

            //letzten Wert zum Vergleich speichern
            lastBrightness = val;
            break;
        }
        //Licht-, Solar- und Innenraumtemperatursensoren
        case IHKA_ADDR:
        {
            //Byte #0 ist der Solarsensor mittig auf dem Armaturenbrett
            /*  
          Solarsensor auf Byte 0: Startet bei 0, in Praller Sonne wurde 73 zuletzt gemeldet. Das ist definitiv der Solar Sensor!
          Dummerweise wir der Sensor manchmal vom Displaygehäuse verdeckt, je nachdem wie die Sonne einfällt. Macht halt das ganze
          Kontrukt teilweise unbruachbar...
      */
            //Byte 4 ist der IR Sensor im Klimabedienteil. Offenbar arbeitet dieser auch mit den Zonensensoren mit.
            break;
        }
        //Steuerung für Helligkeit der Armaturenbeleuchtung (Abblendlicht aktiv)
        case LM_DIM_ADDR:
        {
            //254 = AUS
            //Bereich: 0-253
            //Ab und zu wird 254 einfach so geschickt, wenn 0 vorher aktiv war...warum auch immer
            /*       Serial.print("Beleuchtung (Roh, Ctrl):");
      int dimRawVal = buf[0];
      */
            break;
        }
        //Rückspiegel und dessen Lichtsensorik
        case SPIEGEL_ABBLEND_ADDR:
        {
            break;
        }

        //iDrive Controller

        //iDrive Controller: Drehung
        case IDRIVE_CTRL_ROT_ADDR:
        {
            //Byte 2 beinhaltet den counter
            //Byte 3 Counter Geschwindigkeit der Drehrichtung:
            //        Startet bei 0 bei Drehung im Uhrzeigersinn, wird von 0xFE heruntergezählt bei entgegengesetzter Richtung.
            //Byte 4 0x80 für Drehung im Uhrzeigersinn
            //       0x7F für Drehung gegen den Uhrzeigersinn
            //        Alle anderen Werte: Keine Drehung

            //Code von IAmOrion
            //  https://github.com/IAmOrion/BMW-iDrive-BLE-HID

            //Das System arbeitet nach LittleEndian. Byte 4 und 3 repräsentieren die Drehung und somit auch Drehrichtung.
            /*
      Beispiel:
      E1      FD      AA      FE      7F      1E
      E1      FD      AB      FD      7F      1E
      E1      FD      AC      FE      7F      1E
      E1      FD      AD      FF      7F      1E
      E1      FD      AE      1       80      1E
      E1      FD      AF      2       80      1E
      E1      FD      B0      3       80      1E
      */
            //Es wird also von 80FF nach 7F00 heruntergezählt und umgekehrt.

            byte rotarystepa = buf[3];
            byte rotarystepb = buf[4];
            //unsigned int newpos = (((unsigned int)rotarystepa) + ((unsigned int)rotarystepb) * 0x100);
            //Bitshift Endianness: 0xFF, 0x7F -> 7FFF
            unsigned int newpos = (rotarystepb << 8) + rotarystepa;

            //Initialstellung des Encoders feststellen
            if (!(RotaryInitPositionSet))
            {
                switch (rotarystepb)
                {
                case 0x7F:
                    rotaryposition = (newpos + 1);
                    break;
                case 0x80:
                    rotaryposition = (newpos - 1);
                    break;
                default:
                    rotaryposition = newpos;
                    break;
                }
                RotaryInitPositionSet = true;
            }

            //Da auch die Drehgeschwindigkeit durch byte 3 mit einbezogen wird, sollte diese auch ausgeführt werden.
            //Hier wird einfach das Delta zwischen alter und neuer Position ausgeführt.
            while (rotaryposition < newpos)
            {
                if (iDriveInitSuccess)
                {
                    iDriveRotDir = ROTATION_LEFT;
                    //Während der Menüknopf gedrückt ist (bzw. der Joystick gedrückt ist) kann man mit Drehung des Joysticks blättern
                    if (iDriveBtnPress == BUTTON_LONG_PRESS && lastKnownIdriveButtonType == IDRIVE_BUTTON_CENTER_MENU)
                    {
                        //Taste drücken und ALT gedrückt halten, danach Taste wieder loslassen aber ALT gedrückt halten
                        BPMod->keyboardPress(BP_KEY_LEFT_ARROW, BP_MOD_LEFT_ALT);
                        BPMod->keyboardRelease(BP_KEY_LEFT_ARROW, BP_MOD_NOMOD);
                    }
                    else
                    {
                        BPMod->keyboardPressOnce(BP_KEY_DOWN_ARROW, BP_MOD_NOMOD);
                    }
                }
                rotaryposition++;
            }
            while (rotaryposition > newpos)
            {
                if (iDriveInitSuccess)
                {
                    iDriveRotDir = ROTATION_RIGHT;
                    if (iDriveBtnPress == BUTTON_LONG_PRESS && lastKnownIdriveButtonType == IDRIVE_BUTTON_CENTER_MENU)
                    {
                        //Taste drücken und ALT gedrückt halten, danach Taste wieder loslassen aber ALT gedrückt halten
                        BPMod->keyboardPress(BP_KEY_RIGHT_ARROW, BP_MOD_LEFT_ALT);
                        BPMod->keyboardRelease(BP_KEY_RIGHT_ARROW, BP_MOD_NOMOD);
                    }
                    else
                    {
                        BPMod->keyboardPressOnce(BP_KEY_UP_ARROW, BP_MOD_NOMOD);
                    }
                }
                rotaryposition--;
            }
            //Zeitstempel
            Serial.print(timeStamp + '\t');
            Serial.print("[checkCan] iDrive Rotation: ");
            switch (iDriveRotDir)
            {
            case ROTATION_LEFT:
            {
                Serial.println("Rechtsdrehung");
                break;
            }
            case ROTATION_RIGHT:
            {
                Serial.println("Linksdrehung");
                break;
            }

            default:
            {
                Serial.println("Keine");
                break;
            }
            } //END ROTARY DIRECTION
            break;
        } //END ROTARY

            //Knöpfe und Joystick
        case IDRIVE_CTRL_BTN_ADDR:
        {
            //Dieser Wert erhöht sich, wenn eine Taste gedrückt wurde.
            int buttonCounter = buf[2];

            //Status der Taste (kurz, lang, losgelassen) oder Joystick-Richtung
            int buttonPressType = buf[3];

            //Eingabetyp: Button oder Joystick
            int inputType = buf[4];

            //Knopf
            int buttonType = buf[5];

            //Entprellung der Knöpfe: Bei jedem Tastendruck wird eine Laufnummer auf byte 2 gesendet. Solange diese sich nicht verändert, wird der Knopf gehalten.
            //Zur Sicherheit wird dabei gleichzeitig die ID des Knopfes selbst abgeglichen.
            if ((buttonCounter != previousIdriveButtonPressCounter || lastKnownIdriveButtonPressType != buttonPressType) && previousIdriveButtonTimestamp - currentMillis >= IDRIVE_BUTTON_DEBOUNCE)
            {
                //Fallunterscheidung nach Art des Knopfdrucks:
                // Kurzer Druck = 1 (Wird dauerhaft gesendet)
                // Gehalten = 2 (Wird nach ca 2 Sekunden halten gesendet)
                // Losgelassen = 0 (wird immer nach dem Loslassen gesendet)
                switch (buttonPressType)
                {
                //Kurzer Knopfdruck registriert
                case 0x01:
                {
                    iDriveBtnPress = BUTTON_SHORT_PRESS;
                    break;
                }
                //Lang
                case 0x02:
                {
                    iDriveBtnPress = BUTTON_LONG_PRESS;
                    break;
                }
                } //END BUTONPRESSTYPE
            }
            else
            {
                previousIdriveButtonTimestamp = currentMillis;
                return;
            }

            //Egal wie der vorherige Status war wird beim Senden von "0" die Taste als losgelassen betrachtet.
            if (buttonPressType == 0x00)
            {
                iDriveBtnPress = BUTTON_RELEASE;
                Serial.println("[iDrive] RELEASE");
            }

            //Zeitstempel des letzten Knopfdrucks merken.
            previousIdriveButtonTimestamp = currentMillis;
            //Zuletzt empfangenen Zähler merken.
            previousIdriveButtonPressCounter = buttonCounter;
            //Zuletzt empfangene Bedienungsart merken.
            lastKnownIdriveButtonPressType = buttonPressType;
            //Zuletzt gedrücktne Knopf merken
            lastKnownIdriveButtonType = buttonType;

            //Aussortieren, ob der Knopf in eine Richtung gedrückt wurde oder ob ein Funktionsknopf gedrückt wurde.
            if (inputType != IDRIVE_JOYSTICK)
            {
                printCanMsg(canId, buf, len);
                //Knöpfe entsprechend nach Typ behandeln
                switch (buttonType)
                {
                //Joystick oder Menüknopf
                case IDRIVE_BUTTON_CENTER_MENU:
                {
                    switch (iDriveBtnPress)
                    {
                    //Kurz gedrückt
                    case BUTTON_SHORT_PRESS:
                    {
                        BPMod->keyboardPress(BP_KEY_ENTER, BP_MOD_NOMOD);
                        break;
                    }
                    //Lang gedrückt
                    case BUTTON_LONG_PRESS:
                    {
                        //BPMod->keyboardPress(BP_KEY_F11, BP_MOD_NOMOD);
                        break;
                    }
                    //Losgelassen
                    case BUTTON_RELEASE:
                    {
                        BPMod->keyboardRelease(BP_KEY_ENTER, BP_MOD_NOMOD);
                        break;
                    }
                    } //END BUTTON PRESS DURATION
                    break;
                }
                    //BACK Button
                case IDRIVE_BUTTON_BACK:
                {
                    switch (iDriveBtnPress)
                    {
                    //Kurz gedrückt
                    case BUTTON_SHORT_PRESS:
                    {
                        //Zurück
                        BPMod->keyboardPress(BP_KEY_ESCAPE, BP_MOD_NOMOD);
                        break;
                    }
                    //Lang gedrückt
                    case BUTTON_LONG_PRESS:
                        //BPMod->keyboardPress(BP_KEY_F10, BP_MOD_NOMOD);
                        break;
                    //Losgelassen
                    case BUTTON_RELEASE:
                    {
                        BPMod->keyboardRelease(BP_KEY_ESCAPE, BP_MOD_NOMOD);
                        break;
                    }
                    } //END BUTTON PRESS DURATION
                    break;
                }
                    //OPTION Button
                case IDRIVE_BUTTON_OPTION:
                {
                    switch (iDriveBtnPress)
                    {
                    //Kurz gedrückt
                    case BUTTON_SHORT_PRESS:
                    {
                        //Menü aufrufen
                        BPMod->keyboardPress(BP_KEY_F9, BP_MOD_NOMOD);
                        break;
                    }
                    //Lang gedrückt
                    case BUTTON_LONG_PRESS:
                    {
                        //BPMod->keyboardPress(BP_KEY_F11, BP_MOD_NOMOD);
                        break;
                    }
                    //Losgelassen
                    case BUTTON_RELEASE:
                    {
                        BPMod->keyboardRelease(BP_KEY_F9, BP_MOD_NOMOD);
                        break;
                    }
                    } //END BUTTON PRESS DURATION
                    break;
                }
                    //RADIO Button
                case IDRIVE_BUTTON_RADIO:
                {
                    switch (iDriveBtnPress)
                    {
                    //Kurz gedrückt
                    case BUTTON_SHORT_PRESS:
                    {
                        BPMod->keyboardPress(BP_KEY_F5, BP_MOD_NOMOD);
                        break;
                    }
                    //Lang gedrückt
                    case BUTTON_LONG_PRESS:
                    {
                        break;
                    }
                    //Losgelassen
                    case BUTTON_RELEASE:
                    {
                        BPMod->keyboardRelease(BP_KEY_F5, BP_MOD_NOMOD);
                        break;
                    }

                    } //END BUTTON PRESS DURATION
                    break;
                }
                    //CD Button
                case IDRIVE_BUTTON_CD:
                {
                    switch (iDriveBtnPress)
                    {
                    //Kurz gedrückt
                    case BUTTON_SHORT_PRESS:
                    {
                        BPMod->keyboardPressOnce(BP_KEY_F6, BP_MOD_NOMOD);
                        break;
                    }
                    //Lang gedrückt
                    case BUTTON_LONG_PRESS:
                    {
                        break;
                    }
                    //Losgelassen
                    case BUTTON_RELEASE:
                    {
                        break;
                    }
                    } //END BUTTON PRESS DURATION
                    break;
                }
                    //NAV Button
                case IDRIVE_BUTTON_NAV:
                {
                    switch (iDriveBtnPress)
                    {
                    //Kurz gedrückt
                    case BUTTON_SHORT_PRESS:
                    {
                        BPMod->keyboardPressOnce(BP_KEY_F7, BP_MOD_NOMOD);
                        break;
                    }
                    //Lang gedrückt
                    case BUTTON_LONG_PRESS:
                    {
                        break;
                    }
                    //Losgelassen
                    case BUTTON_RELEASE:
                    {
                        break;
                    }
                    } //END BUTTON PRESS DURATION
                    break;
                }
                    //TEL Button
                case IDRIVE_BUTTON_TEL:
                {
                    switch (iDriveBtnPress)
                    {
                    //Kurz gedrückt
                    case BUTTON_SHORT_PRESS:
                    {
                        BPMod->keyboardPressOnce(BP_KEY_F8, BP_MOD_NOMOD);
                        break;
                    }
                    //Lang gedrückt
                    case BUTTON_LONG_PRESS:
                    {
                        break;
                    }
                    //Losgelassen
                    case BUTTON_RELEASE:
                    {
                        break;
                    }
                    } //END BUTTON PRESS DURATION
                    break;
                }
                default:
                {
                    break;
                } //END BUTTON PRESS
                }
                previousIdriveButton = buttonType;
            }
            else
            {
                switch (buttonPressType)
                {
                    //Hoch (kurz)
                case IDRIVE_JOYSTICK_UP:
                    BPMod->keyboardPressOnce(BP_KEY_UP_ARROW, BP_MOD_NOMOD);
                    break;
                    //Hoch (lang)
                case IDRIVE_JOYSTICK_UP_HOLD:
                    break;
                //Rechts (kurz)
                case IDRIVE_JOYSTICK_RIGHT:
                    BPMod->keyboardPressOnce(BP_KEY_RIGHT_ARROW, BP_MOD_NOMOD);
                    break;
                //Rechts (lang)
                case IDRIVE_JOYSTICK_RIGHT_HOLD:
                    break;
                //Runter (kurz)
                case IDRIVE_JOYSTICK_DOWN:
                    BPMod->keyboardPressOnce(BP_KEY_DOWN_ARROW, BP_MOD_NOMOD);
                    break;
                //Runter (lang)
                case IDRIVE_JOYSTICK_DOWN_HOLD:
                    break;
                //Links (kurz)
                case IDRIVE_JOYSTICK_LEFT:
                    BPMod->keyboardPressOnce(BP_KEY_LEFT_ARROW, BP_MOD_NOMOD);
                    break;
                //Links (lang)
                case IDRIVE_JOYSTICK_LEFT_HOLD:
                    break;
                default:
                    break;
                } //END DIRECTION
            }
            break;
        }

        //PDC 1
        case PDC_ABSTANDSMELDUNG_1_ADDR:
        {
            /*       //Byte 0~3 = Hinten
      //Byte 4~7 = Vorne
      //Angaben in cm von 0 - 255
      //Heck:
      int backOuterLeft = buf[0];
      int backInnerLeft = buf[1];
      int backInnerRight = buf[2];
      int backOuterRight = buf[3];
      //Front:
      int frontOuterLeft = buf[4];
      int frontInnerLeft = buf[5];
      int frontInnerRight = buf[6];
      int frontOuterRight = buf[7];
 */
            break;
        }
        //Rückwärtsgang
        case LM_REVERSESTATUS_ADDR:
        {
            if (buf[0] == 0xFD)
            {
                //Rückwärtsgang NICHT aktiv
            }
            if (buf[0] == 0xFE)
            {
                //Rückwärtsgang AKTIV
            }
            break;
        }
        //Batteriespannung und Status
        case DMEDDE_POWERMGMT_BATTERY_ADDR:
        {
            //(((Byte[1]-240 )*256)+Byte[0])/68
            //float batteryVoltage = (((buf[1] - 240) * 256) + buf[0]) / 68;

            if (buf[3] == 0x00)
            {
                //Zeitstempel
                Serial.print(timeStamp + '\t');
                Serial.println("Engine RUNNING");
            }
            if (buf[3] == 0x09)
            {
                //Zeitstempel
                Serial.print(timeStamp + '\t');
                Serial.println("Engine OFF");
            }
            break;
        }
        //Uhrzeit
        case KOMBI_DATETIME_ADDR:
        {
            //Merken, wann das letzte mal diese Nachricht empfangen wurde.
            previousCanDateTime = millis();
            int hours = 0;
            int minutes = 0;
            int seconds = 0;
            int days = 0;
            int month = 0;
            int year = 0;
            //0: Stunden
            hours = buf[0];
            //1: Minuten
            minutes = buf[1];
            //2: Sekunden
            seconds = buf[2];
            //3: Tage
            days = buf[3];
            //4: die ersten 4 bits stellen den Monat dar.
            month = buf[4] >> 4;
            //6 & 5: Jahr
            // #6 nach links shiften und 5 addieren
            year = (buf[6] << 8) + buf[5];

            //buildtimeStamp();

            break;
        }
        default:
            break;
        }
    }

    //Timeout für Canbus.
    if (currentMillis - previousCanMsgTimestamp >= CAN_TIMEOUT && canbusEnabled == true)
    {
        //Canbus wurde heruntergefahren. Es werden keinerlei Nachrichten mehr seit 30 Sekunden ausgetauscht.
        //Der iDrive Controller ist jetzt als deaktiviert zu betrachten und muss neu intialisiert werden
        iDriveInitSuccess = false;
        canbusEnabled = false;
        //Zeitstempel
        Serial.print(timeStamp + '\t');
        Serial.println("[checkCan] Keine Nachrichten seit 30 Sekunden. Der Bus wird nun als deaktiviert betrachtet.");
    }

    //Aktivitäts-LED ausschalten, wenn Funkstille herrscht.
    if (currentMillis - previousCanMsgTimestamp >= 1000)
    {
        actLedBrightness = 0;
        analogWrite(ACT_LED, actLedBrightness);
    }
}

void printCanMsg(int canId, unsigned char *buffer, int len)
{
    //OUTPUT:
    //ABC FF  FF  FF  FF  FF  FF  FF  FF
    Serial.print('[');
    Serial.print(canId, HEX);
    Serial.print(']');
    Serial.print('\t');
    for (int i = 0; i < len; i++)
    {
        Serial.print(buffer[i], HEX);
        Serial.print("\t");
    }
    Serial.println();
}

void printCanMsgCsv(int canId, unsigned char *buffer, int len)
{
    //OUTPUT:
    //ABC FF  FF  FF  FF  FF  FF  FF  FF
    Serial.print(canId, HEX);
    Serial.print(';');
    Serial.print(len);
    Serial.print(';');
    for (int i = 0; i < len; i++)
    {
        Serial.print(buffer[i], HEX);
        //Semikolon nicht beim letzten Eintrag anhängen
        if (i < len - 1)
        {
            Serial.print(';');
        }
    }
    Serial.println();
}
#pragma endregion

#pragma region "Temperature Control"

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
    temperature = sensors.getTempCByIndex(0);
    sensors.setResolution(firstDeviceAddress, TEMP_RESOLUTION);
    sensors.requestTemperatures();
    //Filter
    temperature = temperature - 0.035 * (temperature - rawtemp);
    //Output
    return temperature;
}
#pragma endregion

#pragma region "Voltage Control"

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
}

#pragma endregion

#pragma region "Connectivity, RTC Sync, ..."
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
    if (wifiStatus != WL_CONNECTED)
    {
        Serial.println("[RTC] Not Connected. Aborting.");
        return;
    }
    sendNTPpacket(timeServer); // send an NTP packet to a time server
    Serial.println("[RTC] request sent");
    // wait to see if a reply is available
    delay(1000);
    if (Udp.parsePacket())
    {
        Serial.println("[RTC] Packet received");
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
unsigned long sendNTPpacket(IPAddress &address)
{
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request

    packetBuffer[0] = 0b11100011; // LI, Version, Mode
    packetBuffer[1] = 0;          // Stratum, or type of clock
    packetBuffer[2] = 6;          // Polling Interval
    packetBuffer[3] = 0xEC;       // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;

    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    Udp.beginPacket(address, 123); //NTP requests are to port 123

    Udp.write(packetBuffer, NTP_PACKET_SIZE);

    Udp.endPacket();
}

void buildtimeStamp()
{
    sprintf(timeStamp, "[%02d:%02d:%02d %02d.%02d.%u]", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds(), rtc.getDay(), rtc.getMonth(), rtc.getYear());
    sprintf(timeString, "%02d:%02d:%02d", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
    sprintf(dateString, "%02d.%02d.%u", rtc.getDay(), rtc.getMonth(), rtc.getYear());
}
#pragma endregion

#pragma region "Fan Control"
void setupPWM()
{
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the 4 PWM channels: timer TCC0 outputs
  const uint8_t CHANNELS = 2;
  const uint8_t pwmPins[] = { 2, 3};   //PB10, PB11
  for (uint8_t i = 0; i < CHANNELS; i++)
  {
    PORT->Group[g_APinDescription[pwmPins[i]].ulPort].PINCFG[g_APinDescription[pwmPins[i]].ulPin].bit.PMUXEN = 1;
  }
  // Connect the TCC0 timer to the port outputs - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
  PORT->Group[g_APinDescription[3].ulPort].PMUX[g_APinDescription[3].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
  // PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
  //PORT->Group[g_APinDescription[5].ulPort].PMUX[g_APinDescription[5].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;    // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  // 20000 = 50Hz, 10000 = 100Hz, 2500  = 400Hz
  REG_TCC0_PER = MAX_FANDUTY;      // Set the frequency of the PWM on TCC0 to 50Hz
  while (TCC0->SYNCBUSY.bit.PER);

  // The CCBx register value corresponds to the pulsewidth in microseconds (us)
  REG_TCC0_CCB0 = MAX_FANDUTY;
  while (TCC0->SYNCBUSY.bit.CCB0);
  REG_TCC0_CCB1 = MAX_FANDUTY;
  while (TCC0->SYNCBUSY.bit.CCB1);

  // Divide the 48MHz signal by 1 giving 48MHz (20.8ns) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

void setFanDutyCycle(int pcCycle)
{
    REG_TCC0_CCB1 = pcCycle;
    while (TCC0->SYNCBUSY.bit.CCB1);
    REG_TCC0_CCB0 = pcCycle;
    while (TCC0->SYNCBUSY.bit.CCB0);
}
#pragma endregion