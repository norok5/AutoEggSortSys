/*
 * EggSorter - Arduino Motor Controller
 * Hardware: Arduino UNO + CNC Shield V3 + A4988 + NEMA-17 (17HS4401S)
 *
 * Benoetigte Bibliotheken (Arduino IDE -> Bibliothek verwalten):
 *   - AccelStepper  (by Mike McCauley)
 *   - Servo         (builtin)
 *
 * Achsenkonfiguration:
 *   X  -> CNC Shield X-Port  (Direktantrieb, GT2-Riemen, 20Z)
 *   Y  -> CNC Shield Y-Port  (linke Fuehrungsschiene, 3:1 Planetengetriebe)
 *   Y2 -> CNC Shield Z-Port  (rechte Fuehrungsschiene, synchron mit Y)
 *   Greifer -> Servo an Pin 11 (Spindle-PWM des CNC-Shield)
 *
 * Protokoll (Raspberry Pi <-> Arduino, 9600 Baud, \n terminiert):
 *   Pi sendet:     Arduino antwortet:
 *   MOVETO x y     OK X:<mm> Y:<mm>    (absolute Position in mm, Ganzzahl)
 *   GRIP           OK                  (Greifer schliessen)
 *   RELEASE        OK                  (Greifer oeffnen)
 *   HOME           OK                  (aktuelle Position als Nullpunkt setzen)
 *   POS            OK X:<mm> Y:<mm>    (aktuelle Position abfragen)
 *   STOP           OK                  (Nothalte mit Verzoegerung)
 *
 *   Beim Start sendet der Arduino: READY
 *   Bei Fehler:                    ERR <Meldung>
 */

#include <AccelStepper.h>
#include <Servo.h>

// ============================================================
// PIN-BELEGUNG  (CNC Shield V3 auf Arduino UNO)
// ============================================================
#define X_STEP_PIN    2
#define X_DIR_PIN     5
#define Y_STEP_PIN    3
#define Y_DIR_PIN     6
#define Y2_STEP_PIN   4   // Z-Port des CNC-Shield -> zweiter Y-Motor
#define Y2_DIR_PIN    7
#define ENABLE_PIN    8   // LOW = alle Treiber aktiv
#define SERVO_PIN     11  // Spindle-PWM -> Greifer-Servo

// ============================================================
// MECHANISCHE PARAMETER
// ============================================================

// NEMA-17 17HS4401S: 1.8 deg/Step = 200 Steps/Umdrehung (Vollschritt)
// Zahnriemen GT2, Riemenscheibe 20 Zaehne: 20 * 2mm = 40 mm/Umdrehung
// X-Achse Direktantrieb:  200 Steps / 40 mm = 5.0 Steps/mm
// Y-Achse 3:1 Getriebe:   200 * 3  / 40 mm = 15.0 Steps/mm

#define X_STEPS_PER_MM   5.0f
#define Y_STEPS_PER_MM   15.0f

// Y2-Motor dreht gegenlaeufig, wenn er spiegelverkehrt montiert ist
// (typisch bei Doppelschlitten-Gantry). true = Richtung umkehren.
#define Y2_INVERTED      false

// Maximalwege (Software-Endanschlag in mm)
#define X_MAX_MM   1500
#define Y_MAX_MM    800

// ============================================================
// GESCHWINDIGKEIT UND BESCHLEUNIGUNG
// ============================================================
// Sanfte Beschleunigung wichtig -> Eier nicht beschaedigen!

#define X_MAX_SPEED_MMS   100.0f    // mm/s
#define X_ACCEL_MMS2      150.0f    // mm/s^2

#define Y_MAX_SPEED_MMS    60.0f    // mm/s (Getriebe -> weniger noetig)
#define Y_ACCEL_MMS2      100.0f    // mm/s^2

// ============================================================
// GREIFER-SERVO
// ============================================================
#define GRIPPER_OPEN_DEG     90    // Winkel Greifer offen
#define GRIPPER_CLOSED_DEG   30    // Winkel Greifer zu  (anpassen!)
#define GRIPPER_SETTLE_MS   600    // Wartezeit nach Greiferbefehl

// ============================================================
// SERIELLE KOMMUNIKATION
// ============================================================
#define BAUD_RATE    9600
#define CMD_BUF_LEN  32

// ============================================================
// GLOBALE OBJEKTE
// ============================================================
AccelStepper motorX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper motorY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper motorY2(AccelStepper::DRIVER, Y2_STEP_PIN, Y2_DIR_PIN);
Servo gripperServo;

char cmdBuf[CMD_BUF_LEN];
byte cmdIdx = 0;

// ============================================================
// HILFSFUNKTIONEN
// ============================================================
long mmToStepsX(int mm)  { return (long)(mm * X_STEPS_PER_MM); }
long mmToStepsY(int mm)  { return (long)(mm * Y_STEPS_PER_MM); }
int  stepsToMmX(long s)  { return (int)(s / X_STEPS_PER_MM); }
int  stepsToMmY(long s)  { return (int)(s / Y_STEPS_PER_MM); }

// Blockierende Fahrt zu absoluter Position (mm).
// X und Y fahren gleichzeitig (kuerzere Gesamtzeit).
void moveToMm(int xMm, int yMm) {
    long xTarget  = mmToStepsX(xMm);
    long yTarget  = mmToStepsY(yMm);
    long y2Target = Y2_INVERTED ? -yTarget : yTarget;

    motorX.moveTo(xTarget);
    motorY.moveTo(yTarget);
    motorY2.moveTo(y2Target);

    while (motorX.distanceToGo() != 0 ||
           motorY.distanceToGo() != 0 ||
           motorY2.distanceToGo() != 0) {
        motorX.run();
        motorY.run();
        motorY2.run();
    }
}

// Position als "OK X:<mm> Y:<mm>" auf Serial ausgeben
void sendPosition() {
    Serial.print("OK X:");
    Serial.print(stepsToMmX(motorX.currentPosition()));
    Serial.print(" Y:");
    Serial.println(stepsToMmY(motorY.currentPosition()));
}

// ============================================================
// BEFEHLS-VERARBEITUNG
// ============================================================
void processCommand(const char* cmd) {

    // --- MOVETO <x_mm> <y_mm> ---
    if (strncmp(cmd, "MOVETO ", 7) == 0) {
        int xMm = 0, yMm = 0;
        if (sscanf(cmd + 7, "%d %d", &xMm, &yMm) != 2) {
            Serial.println("ERR MOVETO: Format = MOVETO <x_mm> <y_mm>");
            return;
        }
        // Software-Endanschlag pruefen
        if (xMm < 0 || xMm > X_MAX_MM || yMm < 0 || yMm > Y_MAX_MM) {
            Serial.println("ERR MOVETO: Position ausserhalb des Fahrbereichs");
            return;
        }
        moveToMm(xMm, yMm);
        sendPosition();
    }

    // --- GRIP ---
    else if (strcmp(cmd, "GRIP") == 0) {
        gripperServo.write(GRIPPER_CLOSED_DEG);
        delay(GRIPPER_SETTLE_MS);
        Serial.println("OK");
    }

    // --- RELEASE ---
    else if (strcmp(cmd, "RELEASE") == 0) {
        gripperServo.write(GRIPPER_OPEN_DEG);
        delay(GRIPPER_SETTLE_MS);
        Serial.println("OK");
    }

    // --- HOME (aktuelle Position als Nullpunkt) ---
    else if (strcmp(cmd, "HOME") == 0) {
        motorX.setCurrentPosition(0);
        motorY.setCurrentPosition(0);
        motorY2.setCurrentPosition(0);
        Serial.println("OK");
    }

    // --- POS ---
    else if (strcmp(cmd, "POS") == 0) {
        sendPosition();
    }

    // --- STOP (Nothalt mit Verzoegerung) ---
    else if (strcmp(cmd, "STOP") == 0) {
        motorX.stop();
        motorY.stop();
        motorY2.stop();
        // Auslaufen lassen (Beschleunigungskurve)
        while (motorX.isRunning() || motorY.isRunning() || motorY2.isRunning()) {
            motorX.run();
            motorY.run();
            motorY2.run();
        }
        Serial.println("OK");
    }

    // --- Unbekannter Befehl ---
    else {
        Serial.print("ERR unbekannter Befehl: ");
        Serial.println(cmd);
    }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    // Alle A4988-Treiber aktivieren (ENABLE aktiv-LOW)
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);

    // X-Achse konfigurieren
    motorX.setMaxSpeed(X_MAX_SPEED_MMS * X_STEPS_PER_MM);
    motorX.setAcceleration(X_ACCEL_MMS2 * X_STEPS_PER_MM);

    // Y-Achse konfigurieren (beide Motoren gleiche Parameter)
    motorY.setMaxSpeed(Y_MAX_SPEED_MMS * Y_STEPS_PER_MM);
    motorY.setAcceleration(Y_ACCEL_MMS2 * Y_STEPS_PER_MM);
    motorY2.setMaxSpeed(Y_MAX_SPEED_MMS * Y_STEPS_PER_MM);
    motorY2.setAcceleration(Y_ACCEL_MMS2 * Y_STEPS_PER_MM);

    // Greifer-Servo initialisieren (geöffnet)
    gripperServo.attach(SERVO_PIN);
    gripperServo.write(GRIPPER_OPEN_DEG);

    Serial.begin(BAUD_RATE);
    Serial.println("READY");
}

// ============================================================
// LOOP - Seriellen Eingabe lesen und Befehle ausfuehren
// ============================================================
void loop() {
    while (Serial.available()) {
        char c = Serial.read();

        // Befehlsende: \n oder \r (Windows: \r\n, Linux: \n)
        if (c == '\n' || c == '\r') {
            if (cmdIdx > 0) {
                cmdBuf[cmdIdx] = '\0';
                processCommand(cmdBuf);
                cmdIdx = 0;
            }
        }
        // Zeichen sammeln (Buffer-Overflow verhindern)
        else if (cmdIdx < CMD_BUF_LEN - 1) {
            cmdBuf[cmdIdx++] = c;
        }
    }
}
