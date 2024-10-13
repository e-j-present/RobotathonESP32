// Bare minimum code for spinning motors triggered by controller input

// Assumes servo is connected to pin 15

// Assumes motor controller IN1 and IN2 are connected to pins 14 and 12

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  !CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <bits/stdc++.h>


#define IN1 12
#define IN2 14

// array containing the pin numbers for motors 
int left_motor[2] = {12, 14}; 
int right_motor[2] = {26, 27};

Servo servo;

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }
}

void setup() {
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();
    
    // getting the servo from pin 15
    servo.attach(15);

    // motor controller outputs
    pinMode(left_motor[0], OUTPUT);
    pinMode(left_motor[1], OUTPUT);

    pinMode(right_motor[0], OUTPUT);
    pinMode(right_motor[1], OUTPUT);
   
    Serial.begin(115200);
}

void loop() {
    BP32.update();
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        GamepadPtr controller = myGamepads[i];
        if (controller && controller->isConnected()) {
           
            if (controller->l1() == 1) {
                Serial.print("Servo move");
                servo.write(1000);
            }
            if (controller->l1() == 0) {
                Serial.print("Servo stop");
                servo.write(1500);
            }
            
            // using the other joystick to control the other axis
            if (controller->axisY() > 0){
                Serial.println(" Right DC motor move");
                digitalWrite(right_motor[0], LOW);
                digitalWrite(right_motor[1], HIGH);
            }

            if (controller->axisY() < 0){
                Serial.println(" Right DC motor move");
                digitalWrite(right_motor[0], HIGH);
                digitalWrite(right_motor[1], LOW);
            }

            if (controller->axisY() == 0){
                Serial.println(" Right DC motor stopped move");
                digitalWrite(right_motor[0], LOW);
                digitalWrite(right_motor[1], LOW);
            }

            

            if(controller->axisRY() > 0) { // negative y is upward on stick
                Serial.println(" Left DC motor move");
                digitalWrite(left_motor[0], LOW);
                digitalWrite(left_motor[1], HIGH);
            }
            if (controller->axisRY() < 0){
                Serial.println(" Left DC motor moving");
                digitalWrite(left_motor[0], HIGH);
                digitalWrite(left_motor[1], LOW);
            }
            if(controller->axisRY() == 0) { // stop motor 1
                Serial.println(" Left DC motor stop");
                digitalWrite(left_motor[0], LOW);
                digitalWrite(left_motor[1], LOW);
            }

            if (controller->axisRX() > 0){
                Serial.println("Moved axis right"); 
            }


            // PHYSICAL BUTTON A
            if (controller->b()) {
                Serial.println("button a pressed");
            }

        }
        vTaskDelay(1);
    }
} 
