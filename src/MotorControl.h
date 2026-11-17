#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include <Wire.h>

//Initialisierung 
void motor_begin();

//Encoder
long motor_encoder(uint8_t motor);
void motor_encoderReset(uint8_t motor);

//Stop Programme
void motor_stop(uint8_t motor);
void motor_stopAll();

//Winkel Ansteuern
void motor_angle(uint8_t motor, float winkel);
void motor_speed(uint8_t motor, int speed);

//Fahren
void motor_drive_speed(int speed);
void motor_drive(float mm);

//Linienfolger
void motor_drive_ir(float mm, int linePosition = 5000);

//Drehen
void motor_turn(float winkel);
void motor_turn_one(uint8_t motor, float winkel);

//Distanz Messen
float motor_distance();
void motor_distanceReset();


#endif