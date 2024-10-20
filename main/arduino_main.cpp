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
#include <Arduino_APDS9960.h>


#define IN1 12
#define IN2 14
#define APDS9960_INT 0
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 100000

#define LED 2




// array containing the pin numbers for motors 
int left_motor[2] = {12, 14}; 
int right_motor[2] = {26, 27};

// red, green, blue
int red_values[3] = {90, 20, 30};
int green_values[3] = {30, 80, 50}; 
int blue_values[3] = {10, 30, 40};

enum Color {
    RED, GREEN, BLUE, NO_COLOR = -1
};

Servo servo;

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

TwoWire I2C_0 = TwoWire(0); 
APDS9960 sensor = APDS9960(I2C_0, APDS9960_INT); 

void blink(uint32_t n){
    // blink on board sensor
    delay(n); 
    digitalWrite(LED, HIGH); 
    delay(n);
    digitalWrite(LED, LOW);  
}

/**
 * Method reads a color and then blinks the on board led light if other colors match the first color
 * push A to start this method and X to stop this method (even though the code says B and Y)
 */
void colorLoop(){
    Serial.println("Color Method starting"); 
    delay(5000); 
    
    while(!sensor.colorAvailable()) {
        // Serial.println("Could not read a color, waiting for 5 seconds"); 
        delay (5);
    }
    int r, g, b, a;
    sensor.readColor(r, g, b, a);
    vTaskDelay(1);

    // read the first color for the first time
    Color color = NO_COLOR; 
    Color reading_color = NO_COLOR; 


    printf("RED value %d, Green Value %d, Blue value %d\n", r, g, b); 
    // reading red color
    if (r >= red_values[0] && g <= red_values[1] && b <= red_values[2]){
        color = RED; 
    }

    // reading green color
    if (r <= green_values[0] && g >= green_values[1] && b <= green_values[2]){
        color = GREEN; 
    }

    if (r <= blue_values[0] && g <= blue_values[1] && b >= blue_values[2]){
        color = BLUE; 
    }

    Serial.printf("Color is %d\n", color); 

    bool looping = true;
    while(looping && color != NO_COLOR){
        while(!sensor.colorAvailable()) {
            // Serial.println("Could not read a color, waiting for 5 seconds"); 
            delay (5);
        }
        sensor.readColor(r, g, b, a);

        // reading red
        if (r >= red_values[0] && g <= red_values[1] && b <= red_values[2]){
            reading_color = RED; 
        }

        // reading green color
        if (r <= green_values[0] && g >= green_values[1] && b <= green_values[2]){
            reading_color = GREEN; 
        }

        if (r <= blue_values[0] && g <= blue_values[1] && b >= blue_values[2]){
            reading_color = BLUE; 
        }


        if (reading_color == color){
            Serial.println("Found Color"); 
            blink(100); 
        }

        BP32.update();
        for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
            GamepadPtr controller = myGamepads[i];
            if (controller && controller->isConnected()) {
        
                // PHYSICAL BUTTON A
                if (controller->y()) {
                    looping = false;  
                    Serial.println("ENDING COLOR METHOD"); 
                }
            }
        }

    }

    
}

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

    // on board led
    pinMode(LED, OUTPUT); 

    I2C_0. begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    sensor.setInterruptPin (APDS9960_INT);
    sensor.begin();
   
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
                // Serial.print("Servo stop");
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
                // Serial.println(" Right DC motor stopped move");
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
                // Serial.println(" Left DC motor stop");
                digitalWrite(left_motor[0], LOW);
                digitalWrite(left_motor[1], LOW);
            }

            if (controller->axisRX() > 0){
                Serial.println("Moved axis right"); 
            }


            // PHYSICAL BUTTON A
            if (controller->b()) {
                Serial.println("B pushed");
                colorLoop(); 
            }

        }
    //     vTaskDelay(1);
    // }


    // while(!sensor.colorAvailable()) {
    //     // Serial.println("Could not read a color, waiting for 5 seconds"); 
    //     delay (5);
    // }
    // int r, g, b, a;
    // sensor.readColor(r, g, b, a);

    

    // Serial.print("a = "); 
    // Serial.print(a); 
    // Serial.print(" r = ");
    // Serial.print(r);
    // Serial.print(" g =");
    // Serial.print(g);
    // Serial.print(" b = ");
    // Serial.println(b);
    // vTaskDelay(1);

    // gain is used to change sensitivity of sensor
    }


} 
