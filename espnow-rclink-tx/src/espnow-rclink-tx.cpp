#include <Arduino.h>
#include "EspNowRcLink/Transmitter.h"
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <stdint.h>
#include <EEPROM.h>


#define MAX_VEL 100
#define ROTOR_RADIUS 0.0225
#define Z_GAIN 0.7

#define DRONE_INDEX 0

#define EEPROM_SIZE 4

unsigned long lastPing;

bool armed = false;
unsigned long timeArmed = 0;

StaticJsonDocument<1024> json;

int xTrim = 0, yTrim = 0, zTrim = 0, yawTrim = 0;

double groundEffectCoef = 28, groundEffectOffset = -0.035;

// nested pid loops
// outer: position pid loop
// inner: velocity pid loop
// velocity pid loop sends accel setpoint to flight controller
double xPosSetpoint = 0, xPos = 0;
double yPosSetpoint = 0, yPos = 0;
double zPosSetpoint = 0, zPos = 0;
double yawPosSetpoint = 0, yawPos, yawPosOutput;

double xyPosKp = 1, xyPosKi = 0, xyPosKd = 0;
double zPosKp = 1.5, zPosKi = 0, zPosKd = 0;
double yawPosKp = 0.3, yawPosKi = 0.1, yawPosKd = 0.05;

double xVelSetpoint, xVel, xVelOutput;
double yVelSetpoint, yVel, yVelOutput;
double zVelSetpoint, zVel, zVelOutput;

double xyVelKp = 0.2, xyVelKi = 0.03, xyVelKd = 0.05;
double zVelKp = 0.3, zVelKi = 0.1, zVelKd = 0.05;

PID xPosPID(&xPos, &xVelSetpoint, &xPosSetpoint, xyPosKp, xyPosKi, xyPosKd, DIRECT);
PID yPosPID(&yPos, &yVelSetpoint, &yPosSetpoint, xyPosKp, xyPosKi, xyPosKd, DIRECT);
PID zPosPID(&zPos, &zVelSetpoint, &zPosSetpoint, zPosKp, zPosKi, zPosKd, DIRECT);
PID yawPosPID(&yawPos, &yawPosOutput, &yawPosSetpoint, yawPosKp, yawPosKi, yawPosKd, DIRECT);

PID xVelPID(&xVel, &xVelOutput, &xVelSetpoint, xyVelKp, xyVelKi, xyVelKd, DIRECT);
PID yVelPID(&yVel, &yVelOutput, &yVelSetpoint, xyVelKp, xyVelKi, xyVelKd, DIRECT);
PID zVelPID(&zVel, &zVelOutput, &zVelSetpoint, zVelKp, zVelKi, zVelKd, DIRECT);


unsigned long lastLoopTime = micros();

float loopFrequency = 2000.0;

#ifdef lolin32
#define BUILTIN_LED 8
#elif ESP32-C3-SuperMini

#define BUILTIN_LED 8
#endif

// uncomment to print some details to console
#define PRINT_INFO

EspNowRcLink::Transmitter tx;

char buffer[1024];
int ch[8];

// callback function that will be executed when data is received
void OnDataRecv(const char *incomingData) {
  // Serial.println((char*)incomingData);
  int len = strlen((char *)incomingData);
  DeserializationError err = deserializeJson(json, (char *)incomingData);

  if (err) {
    Serial.print("failed to parse json");
    return;
  }

  if (json.containsKey("pos") && json.containsKey("vel")) {
    xPos = json["pos"][0];
    yPos = json["pos"][1];
    zPos = json["pos"][2];
    yawPos = json["pos"][3];

    xVel = json["vel"][0];
    yVel = json["vel"][1];
    zVel = json["vel"][2];
  } else if (json.containsKey("armed")) {
    if (json["armed"] != armed && json["armed"]) {
      timeArmed = millis();
    }
    armed = json["armed"];
  } else if (json.containsKey("setpoint")) {
    xPosSetpoint = json["setpoint"][0];
    yPosSetpoint = json["setpoint"][1];
    zPosSetpoint = json["setpoint"][2];
  } else if (json.containsKey("pid")) {
    xPosPID.SetTunings(json["pid"][0], json["pid"][1], json["pid"][2]);
    yPosPID.SetTunings(json["pid"][0], json["pid"][1], json["pid"][2]);
    zPosPID.SetTunings(json["pid"][3], json["pid"][4], json["pid"][5]);
    yawPosPID.SetTunings(json["pid"][6], json["pid"][7], json["pid"][8]);

    xVelPID.SetTunings(json["pid"][9], json["pid"][10], json["pid"][11]);
    yVelPID.SetTunings(json["pid"][9], json["pid"][10], json["pid"][11]);
    zVelPID.SetTunings(json["pid"][12], json["pid"][13], json["pid"][14]);

    groundEffectCoef = json["pid"][15];
    groundEffectOffset = json["pid"][16];
  } else if (json.containsKey("trim")) {
    xTrim = json["trim"][0];
    yTrim = json["trim"][1];
    zTrim = json["trim"][2];
    yawTrim = json["trim"][3];
  }

  lastPing = micros();
}

void resetPid(PID &pid, double min, double max) {
  pid.SetOutputLimits(0.0, 1.0); 
  pid.SetOutputLimits(-1.0, 0.0);
  pid.SetOutputLimits(min, max);
}



void setup()
{
  pinMode(BUILTIN_LED, OUTPUT);
  Serial.begin(115200);
  #ifdef PRINT_INFO
  Serial.println("EspNowRcLink Transmitter");
  #endif
  tx.begin(true);
  xPosPID.SetMode(AUTOMATIC);
  yPosPID.SetMode(AUTOMATIC);
  zPosPID.SetMode(AUTOMATIC);
  yawPosPID.SetMode(AUTOMATIC);
  xVelPID.SetMode(AUTOMATIC);
  yVelPID.SetMode(AUTOMATIC);
  zVelPID.SetMode(AUTOMATIC);
  
  // Sample rate is determined by main loop
  xPosPID.SetSampleTime(0);
  yPosPID.SetSampleTime(0);
  zPosPID.SetSampleTime(0);
  yawPosPID.SetSampleTime(0);
  xVelPID.SetSampleTime(0);
  yVelPID.SetSampleTime(0);
  zVelPID.SetSampleTime(0);

  xPosPID.SetOutputLimits(-MAX_VEL, MAX_VEL);
  yPosPID.SetOutputLimits(-MAX_VEL, MAX_VEL);
  zPosPID.SetOutputLimits(-MAX_VEL, MAX_VEL);
  yawPosPID.SetOutputLimits(-1, 1);
  xVelPID.SetOutputLimits(-1, 1);
  yVelPID.SetOutputLimits(-1, 1);
  zVelPID.SetOutputLimits(-1, 1);

  EEPROM.begin(EEPROM_SIZE);

  // xTrim = EEPROM.read(0);
  // yTrim = EEPROM.read(1);
  // zTrim = EEPROM.read(2);
  // yawTrim = EEPROM.read(3);

  lastPing = micros();
  lastLoopTime = micros();

}

void loop()
{
  vTaskDelay(1);
  uint32_t now = micros();
  static int v = 0;
  static uint32_t delta = 0;
  static bool ledState = 0;
  static bool connected = false;
  static uint32_t lastChannel = 32;
  int availableBytes = Serial.available();

  if (availableBytes) {
    int droneIndex = Serial.read() - '0';
    Serial.readBytes(buffer, availableBytes-1);
    buffer[availableBytes-1] = '\0';
    Serial.printf("\n drone index %d: ", droneIndex);
    Serial.print(buffer);
    OnDataRecv(buffer);
  }
  if (micros() - lastPing > 2e6) {
    armed = false;
  }
  if (armed) {
    ch[4] = 1800;
  } else {
    ch[4] = 172;
    resetPid(xPosPID, -MAX_VEL, MAX_VEL);
    resetPid(yPosPID, -MAX_VEL, MAX_VEL);
    resetPid(zPosPID, -MAX_VEL, MAX_VEL);
    resetPid(yawPosPID, -1, 1);
    resetPid(xVelPID, -1, 1);
    resetPid(yVelPID, -1, 1);
    resetPid(zVelPID, -1, 1);
  }

  xPosPID.Compute();
  yPosPID.Compute();
  zPosPID.Compute();
  yawPosPID.Compute();

  xVelPID.Compute();
  yVelPID.Compute();
  zVelPID.Compute();

  int xPWM = 992 + (xVelOutput * 811) + xTrim;
  int yPWM = 992 + (yVelOutput * 811) + yTrim;
  int zPWM = 992 + (Z_GAIN * zVelOutput * 811) + zTrim;
  int yawPWM = 992 + (yawPosOutput * 811) + yawTrim;
  double groundEffectMultiplier = 1 - groundEffectCoef*pow(((2*ROTOR_RADIUS) / (4*(zPos-groundEffectOffset))), 2);
  zPWM *= max(0., groundEffectMultiplier);
  zPWM = armed && millis() - timeArmed > 100 ? zPWM : 172;
  ch[0] = -yPWM;
  ch[1] = xPWM;
  ch[2] = zPWM;
  ch[3] = yawPWM;


    for(size_t c = 0; c < 8; c++)
    {
      const int16_t val = ch[c];
      tx.setChannel(c, val);
      if(c == 2) v = val;
    }
    tx.commit();
    tx.update();
static uint32_t connectedNext = now + 50000;
 if(now >= connectedNext){
  if(WiFi.channel() == lastChannel)
    {
      connected = true;
      digitalWrite(BUILTIN_LED, 0);
      #ifdef PRINT_INFO
      Serial.printf("Connected on channel %d\n", WiFi.channel());
      #endif
    } 
  else
    {
    
    lastChannel = WiFi.channel();
    connected = false;
    digitalWrite(BUILTIN_LED, ledState);
    ledState = !ledState; 
  }
  connectedNext = now + 50000;
}
   
#ifdef PRINT_INFO
  static uint32_t printNext = now + 500000;
  if(now >= printNext)
  {
    Serial.printf("V: %d, P: %d, D: %d, C: %d, S: %d\n", v, ch[2], delta / 100, WiFi.channel(),WiFi.localIP());
    Serial.printf("PWM x: %d, y: %d, z: %d, yaw: %d\nPos x: %f, y: %f, z: %f, yaw: %f\n", xPWM, yPWM, zPWM, yawPWM, xVel, yVel, zPos, yawPos);
    Serial.printf("Setpoint x: %f, y: %f, z: %f\n", xVelSetpoint, yVelSetpoint, zVelSetpoint);
    Serial.printf("Pos x: %f, y: %f, z: %f\n", xVel, yVel, zPos);
    Serial.printf("Output x: %f, y: %f, z: %f\n", xVelOutput, yVelOutput, zVelOutput);
    printNext = now + 500000;
  }
#else
  (void)v;
  (void)delta;
#endif
}

