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
#include <QTRSensors.h>


#define IN1 12
#define IN2 14
#define APDS9960_INT 0
#define I2C_SDA 22
#define I2C_SCL 21
#define I2C_FREQ 100000

#define LED 2



#define FRONT_DISTANCE_SENSOR 25
#define LEFT_DISTANCE_SENSOR 33
#define RIGHT_DISTANCE_SENSOR 32
#define SERVO_MIN_SPEED 70
#define SERVO_MAX_SPEED 110
//
#define LED_PIN 13

#define LEFT_LINE_SENSOR 36
#define MIDDLE_LINE_SENSOR 39
#define RIGHT_LINE_SENSOR 34





// array containing the pin numbers for motors 
int left_motor[2] = {14, 12}; 
int right_motor[2] = {26, 27};

// red, green, blue
int red_values[3] = {10, 10, 10};
int green_values[3] = {14, 10, 9}; 
int blue_values[3] = {10, 15, 12};
int servo_pos = 90; 

enum Color {
    RED, GREEN, BLUE, NO_COLOR = -1
};

Servo servo;

QTRSensors qtr;
QTRSensors line_qtr;

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

void spin_left(){
    digitalWrite(right_motor[0], LOW);  
    digitalWrite(right_motor[1], LOW);
    digitalWrite(left_motor[0], LOW); 
    digitalWrite(left_motor[1], LOW);
    delay(500); 


    
    digitalWrite(right_motor[0], LOW); 
    digitalWrite(right_motor[1], HIGH);

    

    digitalWrite(left_motor[0], LOW); 
    digitalWrite(left_motor[1], LOW);
    delay(375); 
}

void spin_right(){
    digitalWrite(right_motor[0], LOW); 
    digitalWrite(right_motor[1], LOW);
    digitalWrite(left_motor[0], LOW); 
    digitalWrite(left_motor[1], LOW);

    delay(500); 

    digitalWrite(right_motor[0], HIGH); 
    digitalWrite(right_motor[1], LOW);

    digitalWrite(left_motor[0], LOW); 
    digitalWrite(left_motor[1], LOW);
    delay(375); 
}
void go_fowrward(){
    digitalWrite(left_motor[0], HIGH); 
    digitalWrite(left_motor[1], LOW); 


    digitalWrite(right_motor[0], HIGH); 
    digitalWrite(right_motor[1], LOW);

}
void stop(){
    digitalWrite(left_motor[0], LOW); 
    digitalWrite(left_motor[1], LOW); 


    digitalWrite(right_motor[0], LOW); 
    digitalWrite(right_motor[1], LOW);

}

void distanceLoop(){
    Serial.println("Distance method starting");

    bool looping = true;
    uint16_t sensorValues[3];
    while(looping){
        
        qtr.read(sensorValues);
        uint16_t front= sensorValues[0]; 
        uint16_t left = sensorValues[1]; 
        uint16_t right = sensorValues[2];


        /*method goes here*/
        // while the front distance sensor is less than 2500, go straight
        if(front <= 2000){
            go_fowrward(); 
        } 
        
        // spin right
        else if (right <= 1000){
            // stopping
            spin_right();  
            
            //
        }
        
        // otherwise spin left
        else{
            spin_left(); 
        }




        BP32.update();
        for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
            GamepadPtr controller = myGamepads[i];
            if (controller && controller->isConnected()) {
        
                // PHYSICAL BUTTON A
                if (controller->y()) {
                    looping = false;  
                    Serial.println("ENDING DISTANCE METHOD"); 
                }
            }
        }

    }
}

/**
 * Method reads a color and then blinks the on board led light if other colors match the first color
 * push A to start this method and X to stop this method (even though the code says B and Y)
 */
void colorLoop(){
    Serial.println("Color Method starting"); 
    
    delay(250); 
    
    while(!sensor.colorAvailable()) {
        // Serial.println("Could not read a color, waiting for 5 seconds"); 
        delay (5);
    }
    int starting_r, starting_g, starting_b;
    digitalWrite(LED_PIN, HIGH); 
    sensor.readColor(starting_r, starting_g, starting_b);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
 
    Serial.printf("RED value %d, Green Value %d, Blue value %d\n", starting_r, starting_g, starting_b); 

    int threshold = 15; 
    int r, g, b;
    sensor.readColor(r, g, b); 
    while( (_abs(starting_r-r) <= threshold) && (_abs(starting_g-g) <= threshold) && (_abs(starting_b-b) <= threshold)){
        go_fowrward();
    }
    stop(); 
    delay(1000); 

    while (!((_abs(starting_r-r) <= threshold) && (_abs(starting_g-g) <= threshold) && (_abs(starting_b-b) <= threshold))){
        while(!sensor.colorAvailable()) {
            // Serial.println("Could not read a color, waiting for 5 seconds"); 
            delay (5);
        }

        go_fowrward(); 
        sensor.readColor(r, g, b);
        Serial.printf("RED value %d, Green Value %d, Blue value %d\n", r, g, b); 

        BP32.update();
        for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
            GamepadPtr controller = myGamepads[i];
            if (controller && controller->isConnected()) {
        
                // PHYSICAL BUTTON A
                if (controller->y()) {
                    Serial.println("ENDING COLOR METHOD"); 
                    break; 
                }
            }
        }
    }


    
}

void line_loop() {
    bool looping = true;
    while (looping) {
        uint16_t sensor_line_values[3];
        line_qtr.read(sensor_line_values);
        uint16_t left_line = sensor_line_values[0]; 
        uint16_t middle_line = sensor_line_values[1]; 
        uint16_t right_line = sensor_line_values[2];
        int16_t position = line_qtr.readLineBlack(sensor_line_values);
        Serial.println(position);
        // go_fowrward();
        if (position == 0){
            digitalWrite(LED_PIN, HIGH); 
        }

        if(position >= 300 && position <= 1700){
            go_fowrward(); 
        } 

        // spin right
        else if (position < 300){
            // stopping
            spin_left();

            //
        }

        // otherwise spin left
        else{
            spin_right(); 
        }

        BP32.update();
        for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
            GamepadPtr controller = myGamepads[i];
            if (controller && controller->isConnected()) {
                if (controller->y()) {
                    looping = false;
                    Serial.println("ENDING LINE METHOD");
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
    servo.setPeriodHertz(50);
    servo.attach(15, 1000, 2000);
    servo.write(servo_pos);

    // motor controller outputs
    pinMode(left_motor[0], OUTPUT);
    pinMode(left_motor[1], OUTPUT);

    pinMode(right_motor[0], OUTPUT);
    pinMode(right_motor[1], OUTPUT);

    pinMode(LED_PIN, OUTPUT); 

    // on board led
    pinMode(LED, OUTPUT); 

    I2C_0. begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    sensor.setInterruptPin (APDS9960_INT);
    sensor.begin();

    qtr.setTypeAnalog(); 
    qtr.setSensorPins((const uint8_t[]){FRONT_DISTANCE_SENSOR, LEFT_DISTANCE_SENSOR, RIGHT_DISTANCE_SENSOR}, 3);


    line_qtr.setTypeAnalog();
    line_qtr.setSensorPins((const uint8_t[]){LEFT_LINE_SENSOR, MIDDLE_LINE_SENSOR, RIGHT_LINE_SENSOR}, 3);
    for (uint8_t i = 0; i < 250; i++) {
        // Serial.println("calibrating");
        line_qtr.calibrate();
        delay(20);
    }


   
    Serial.begin(115200);
}

void loop() {
    BP32.update();
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        GamepadPtr controller = myGamepads[i];
        if (controller && controller->isConnected()) {
           
            //servo.write(1500);

            if (controller->l1() == 1) {
                // Serial.printf("Servo move towards robot: %d\n", servo_pos);
                servo_pos += 1; 
                if (servo_pos > SERVO_MAX_SPEED){
                    servo_pos = SERVO_MAX_SPEED; 
                }
                servo.write(servo_pos);
            }

            else if (controller->r1() == 1){
                // Serial.printf("Move servo towards ground: %d\n", servo_pos); 
                servo_pos -= 1; 
                if (servo_pos < SERVO_MIN_SPEED){
                    servo_pos = SERVO_MIN_SPEED; 
                }
                servo.write(servo_pos);
            } else{
                // Serial.printf("Not moving servo: %d\n", servo_pos);
                if (servo_pos < 90){
                    for(; servo_pos <= 90; servo_pos += 1){
                        servo.write(servo_pos); 
                        delay(25); 
                    }
                }
                else if (servo_pos > 90){
                    for(; servo_pos >= 90; servo_pos -= 1){
                        // printf("Servo value: %d\n", servo.read()); 
                        servo.write(servo_pos); 
                        delay(25); 
                    }
                }
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
                blink(1000); 
                colorLoop(); 
            }

            // PHYSICAL BUTTON B
            if (controller->a()){
                Serial.println("A Pushed"); 
                distanceLoop(); 
            }

            // PHYSICAL BUTTON Y
            if (controller->x()) {
                Serial.println("X Pushed");
                line_loop();
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

    // uint16_t sensorValues[3];
    // qtr.read(sensorValues);

    // printf("Distance is: Front - %d, Left - %d, Right - %d\n", sensorValues[0], sensorValues[1], sensorValues[2]); 

    // delay(500); 

    // digitalWrite(WHITELED, HIGH);

    // for (int pos = 0; pos <= 180; pos += 1){
    //     servo.write(pos); 
    //     delay(15);
    // }
    // for (int pos = 180; pos >= 0; pos -= 1){
    //     servo.write(pos); 
    //     delay(15);
    // }
    
    // digitalWrite(LED_PIN, HIGH); 
    // delay(500);
    // colorLoop(); 
    // digitalWrite(LED_PIN, LOW); 


} 
