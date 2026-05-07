#include <ESP32Servo.h>

// ===== Lid Sensor =====
#define TRIG1 5
#define ECHO1 18

// ===== Trash Level Sensor =====
#define TRIG2 19
#define ECHO2 21

// ===== Servo =====
#define SERVO_PIN 13

// ===== Buzzer =====
#define BUZZER 23

Servo lidServo;

// ===== Variables =====
float distance1, distance2;
bool lidOpen = false;

// ===== Millis Timers =====
unsigned long lastSensorRead  = 0;
unsigned long buzzerStartTime = 0;
unsigned long noObjectTime    = 0;  // tracks when object left

// ===== Industrial Tuning Parameters =====
#define SENSOR_READ_INTERVAL    200
#define LID_CONFIRM_COUNT       3
#define LID_CLEAR_COUNT         4
#define BIN_FULL_CONFIRM_COUNT  5
#define BIN_CLEAR_CONFIRM_COUNT 8
#define BUZZER_BEEP_ON          500
#define BUZZER_BEEP_OFF         10000
#define LID_OBJECT_THRESHOLD    30.0
#define BIN_FULL_THRESHOLD      6.0
#define LID_CLOSE_DELAY         3000  // ms after object leaves to close lid

// ===== Debounce Counters =====
int lidDetectCount = 0;
int lidClearCount  = 0;
int binFullCount   = 0;
int binClearCount  = 0;

// ===== State Flags =====
bool binFull         = false;
bool buzzerBeepOn    = false;
bool lidCooldown     = false;
bool objectPresent   = false;  // true while someone is in front
bool waitingToClose  = false;  // true during the 3sec countdown

float readDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 23200);
  if (duration == 0) return 999;

  return duration * 0.034 / 2;
}

float readDistanceAvg(int trigPin, int echoPin)
{
  float r1 = readDistance(trigPin, echoPin);
  delayMicroseconds(60);
  float r2 = readDistance(trigPin, echoPin);
  delayMicroseconds(60);
  float r3 = readDistance(trigPin, echoPin);

  float sum = 0;
  int count = 0;

  if (r1 != 999) { sum += r1; count++; }
  if (r2 != 999) { sum += r2; count++; }
  if (r3 != 999) { sum += r3; count++; }

  if (count == 0) return 999;
  return sum / count;
}

void handleLid(float dist)
{
  bool detected = (dist < LID_OBJECT_THRESHOLD && dist != 999);

  if (detected)
  {
    // ===== Object is present =====
    lidClearCount = 0;
    waitingToClose = false; // cancel any close countdown

    if (!lidOpen && !lidCooldown)
    {
      lidDetectCount++;

      if (lidDetectCount >= LID_CONFIRM_COUNT)
      {
        lidDetectCount = 0;
        lidOpen        = true;
        objectPresent  = true;
        lidServo.write(90);
        Serial.println("[LID] Opening — Object confirmed");
      }
    }
    else if (lidOpen)
    {
      // Already open — keep resetting, person still there
      objectPresent = true;
    }
  }
  else
  {
    // ===== Object gone =====
    lidDetectCount = 0;

    if (lidOpen)
    {
      if (!waitingToClose)
      {
        // Start the 3 second countdown
        waitingToClose = true;
        noObjectTime   = millis();
        Serial.println("[LID] Object left — Starting 3s close countdown");
      }
      else if (millis() - noObjectTime >= LID_CLOSE_DELAY)
      {
        // 3 seconds passed with no object — close lid
        lidOpen        = false;
        lidCooldown    = true;
        waitingToClose = false;
        objectPresent  = false;
        lidClearCount  = 0;
        lidServo.write(0);
        Serial.println("[LID] Closing — No object for 3s");
      }
    }

    // ===== Cooldown clear =====
    if (lidCooldown)
    {
      lidClearCount++;
      if (lidClearCount >= LID_CLEAR_COUNT)
      {
        lidClearCount = 0;
        lidCooldown   = false;
        Serial.println("[LID] Cooldown cleared — Ready for next trigger");
      }
    }
  }
}

void handleBinFull(float dist)
{
  if (dist == 999) return;

  if (dist < BIN_FULL_THRESHOLD)
  {
    binClearCount = 0;
    binFullCount++;

    if (!binFull && binFullCount >= BIN_FULL_CONFIRM_COUNT)
    {
      binFull         = true;
      buzzerBeepOn    = false;
      buzzerStartTime = 0;
      Serial.println("[BIN] FULL — Alert ON");
    }
  }
  else
  {
    binFullCount = 0;
    binClearCount++;

    if (binFull && binClearCount >= BIN_CLEAR_CONFIRM_COUNT)
    {
      binFull       = false;
      binClearCount = 0;
      digitalWrite(BUZZER, LOW);
      buzzerBeepOn  = false;
      Serial.println("[BIN] Cleared — Alert OFF");
    }
  }

  if (binFull)
  {
    unsigned long elapsed = millis() - buzzerStartTime;

    if (!buzzerBeepOn && elapsed >= BUZZER_BEEP_OFF)
    {
      digitalWrite(BUZZER, HIGH);
      buzzerBeepOn    = true;
      buzzerStartTime = millis();
    }
    else if (buzzerBeepOn && elapsed >= BUZZER_BEEP_ON)
    {
      digitalWrite(BUZZER, LOW);
      buzzerBeepOn    = false;
      buzzerStartTime = millis();
    }
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);

  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  lidServo.attach(SERVO_PIN);
  lidServo.write(0);

  Serial.println("=============================");
  Serial.println("   Smart Bin System Started  ");
  Serial.println("=============================");
}

void loop()
{
  unsigned long now = millis();

  if (now - lastSensorRead >= SENSOR_READ_INTERVAL)
  {
    lastSensorRead = now;

    distance1 = readDistanceAvg(TRIG1, ECHO1);
    distance2 = readDistanceAvg(TRIG2, ECHO2);

    Serial.print("[SENSOR] Object: ");
    Serial.print(distance1);
    Serial.print(" cm | Trash Level: ");
    Serial.print(distance2);
    Serial.println(" cm");

    handleLid(distance1);
    handleBinFull(distance2);
  }
}