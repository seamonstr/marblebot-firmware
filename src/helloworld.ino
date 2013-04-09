#include "Arduino.h"
#include "Servo.h"

// Include application, user and local libraries
#include "Memory.h"
#include "Sensors.h"
#include "MarblebotSensors.h"

void setup() {
    sensorSetup();
    Serial.println("Marblebot is go!");
}


void loop() {
    poll();
}

