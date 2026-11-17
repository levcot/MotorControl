
#include "MotorControl.h"
#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include "IMU.h"
#include "IR.h"

// --------------------------------------------------
// ENCODER
// --------------------------------------------------

static const uint8_t ENCODER_PINS_A[6] ={1, 3, 5, 7, 9, 11};
static const uint8_t ENCODER_PINS_B[6] ={2, 4, 6, 8, 10, 12};

static volatile int32_t encoderCounts[6] = {0};

static inline void readEncoder(uint8_t index)
{
    if (digitalRead(ENCODER_PINS_B[index]) == HIGH)
    {
        encoderCounts[index]++;
    }
    else
    {
        encoderCounts[index]--;
    }
}

static void encoder1ISR() {readEncoder(0);}

static void encoder2ISR() {readEncoder(1);}

static void encoder3ISR() {readEncoder(2);}
 
static void encoder4ISR() {readEncoder(3);}

static void encoder5ISR() {readEncoder(4);}

static void encoder6ISR() {readEncoder(5);}

// --------------------------------------------------
// STOPP PROGRAMME
// --------------------------------------------------

static constexpr int MOTOR_MAX_SPEED = 800;
static constexpr int MOTOR_DEFAULT_SPEED = 300;

static int motorSpeeds[6] =
{
    MOTOR_DEFAULT_SPEED,
    MOTOR_DEFAULT_SPEED,
    MOTOR_DEFAULT_SPEED,
    MOTOR_DEFAULT_SPEED,
    MOTOR_DEFAULT_SPEED,
    MOTOR_DEFAULT_SPEED
};

// --------------------------------------------------
// WINKEL ANSTEUERN
// --------------------------------------------------

static constexpr float ANGLE_COUNTS_PER_REVOLUTION = 12.0f * 379.17f;

static MotoronI2C motoron2(17);
static MotoronI2C motoron3(18);

static MotoronI2C* getMotoronAngle(uint8_t motor)
{
    if (motor < 3 || motor > 6)
    {
        return nullptr;
    }

    if (motor <= 4) return &motoron2;

    return &motoron3;
}

static uint8_t getMotorChannelAngle(uint8_t motor)
{
    return ((motor - 1) % 2) + 1;
}

void motor_angle_run(uint8_t motor, int speed)
{
    if (motor < 3 || motor > 6)
    {
        return;
    }

    int maxSpeed = abs(motorSpeeds[motor - 1]);

    speed = constrain(speed,-maxSpeed,maxSpeed);

    MotoronI2C* controller = getMotoronAngle(motor);

    controller->setSpeedNow(getMotorChannelAngle(motor),speed);
}


// --------------------------------------------------
// FAHREN  
// --------------------------------------------------

static MotoronI2C motoron1(16);

static constexpr float DRIVE_COUNTS_PER_REVOLUTION = 48.0f * 9.68f;

static constexpr float WHEEL_DIAMETER_MM = 60.0f;
static constexpr float AXLE_TRACK_MM = 196.5f;

static MotoronI2C* getMotoronDriveIMU(uint8_t motor)
{
    if (motor < 1 || motor > 2)
    {
        return nullptr;
    }

    return &motoron1;
}

static uint8_t getMotorChannelDriveIMU(uint8_t motor)
{
    return ((motor - 1) % 2) + 1;
}

void motor_drive_run_IMU(float turn_rate, float mm)
{
    int maxSpeedLeft = (mm > 0.0f)
        ? motorSpeeds[0]
        : -motorSpeeds[0];

    int maxSpeedRight = (mm > 0.0f)
        ? -motorSpeeds[1]
        : motorSpeeds[1];

    float omega = turn_rate * PI / 180.0f;

    float change = MOTOR_MAX_SPEED / ((MOTOR_MAX_SPEED * WHEEL_DIAMETER_MM * PI / 60.0f) / (omega * AXLE_TRACK_MM / 2.0f));

    float maxChange = MOTOR_MAX_SPEED - motorSpeeds[0];

    change = constrain(change,-maxChange,maxChange);

    float leftSpeed = maxSpeedLeft - change;
    float rightSpeed = maxSpeedRight + change;

    leftSpeed = constrain(leftSpeed,-MOTOR_MAX_SPEED,MOTOR_MAX_SPEED);
    rightSpeed = constrain(rightSpeed,-MOTOR_MAX_SPEED,MOTOR_MAX_SPEED);

    MotoronI2C* controller1 = getMotoronDriveIMU(1);
    MotoronI2C* controller2 = getMotoronDriveIMU(2);

    controller1->setSpeedNow(getMotorChannelDriveIMU(1), (int)leftSpeed);
    controller2->setSpeedNow(getMotorChannelDriveIMU(2), (int)rightSpeed);
}

// --------------------------------------------------
// LINIENFOLGER 
// --------------------------------------------------

static MotoronI2C* getMotoronDriveIR(uint8_t motor)
{
    if (motor < 1 || motor > 2)
    {
        return nullptr;
    }

    return &motoron1;
}

static uint8_t getMotorChannelDriveIR(uint8_t motor)
{
    return ((motor - 1) % 2) + 1;
}

void motor_drive_run_IR(float turn_rate, float mm)
{
    int maxSpeedLeft = (mm > 0.0f)
        ? motorSpeeds[0]
        : -motorSpeeds[0];

    int maxSpeedRight = (mm > 0.0f)
        ? -motorSpeeds[1]
        : motorSpeeds[1];

    float omega = turn_rate * PI / 180.0f;

    //Umrechnung von mm/s in PWM Wert; anteil an 800 
    float change = MOTOR_MAX_SPEED / ((MOTOR_MAX_SPEED * WHEEL_DIAMETER_MM * PI / 60.0f) / (omega * AXLE_TRACK_MM / 2.0f));

    float maxChange = MOTOR_MAX_SPEED - motorSpeeds[0];

    change = constrain(change,-maxChange,maxChange);

    float leftSpeed = maxSpeedLeft - change;
    float rightSpeed = maxSpeedRight + change;

    MotoronI2C* controller1 = getMotoronDriveIR(1);
    MotoronI2C* controller2 = getMotoronDriveIR(2);

    controller1->setSpeedNow(getMotorChannelDriveIR(1), (int)leftSpeed);
    controller2->setSpeedNow(getMotorChannelDriveIR(2), (int)rightSpeed);
}

// --------------------------------------------------
// DREHEN 
// --------------------------------------------------

static MotoronI2C* getMotoronTurn(uint8_t motor)
{
    if (motor < 1 || motor > 2)
    {
        return nullptr;
    }

    return &motoron1;
}

static uint8_t getMotorChannelTurn(uint8_t motor)
{
    return ((motor - 1) % 2) + 1;
}

void motor_turn_run(float turn_rate, float winkel)
{
    int maxSpeedLeft = (winkel > 0.0f)
        ? motorSpeeds[0]
        : -motorSpeeds[0];

    int maxSpeedRight = (winkel > 0.0f)
        ? motorSpeeds[1]
        : -motorSpeeds[1];

    float omega = turn_rate * PI / 180.0f;

    //Umrechnung von mm/s in PWM Wert; anteil an 800 
    float change = MOTOR_MAX_SPEED / ((MOTOR_MAX_SPEED * WHEEL_DIAMETER_MM * PI / 60.0f) / (omega * AXLE_TRACK_MM / 2.0f));

    float maxChange = MOTOR_MAX_SPEED - motorSpeeds[0];

    change = constrain(change,-maxChange,maxChange);

    float leftSpeed = maxSpeedLeft - change;
    float rightSpeed = maxSpeedRight + change;

    MotoronI2C* controller1 = getMotoronTurn(1);
    MotoronI2C* controller2 = getMotoronTurn(2);

    controller1->setSpeedNow(getMotorChannelTurn(1), (int)leftSpeed);
    controller2->setSpeedNow(getMotorChannelTurn(2), (int)rightSpeed);
}

void motor_turn_one_run(float turn_rate, uint8_t motor, float winkel)
{
    if (motor < 1 || motor > 2)
    {
        return;
    }

    float omega = turn_rate * PI / 180.0f;

    //Umrechnung von mm/s in PWM Wert; anteil an 800 
    float speed = MOTOR_MAX_SPEED / ((MOTOR_MAX_SPEED * WHEEL_DIAMETER_MM * PI / 60.0f) / (omega * AXLE_TRACK_MM / 2.0f));

    speed = (winkel > 0.0f)
        ? speed
        : -speed;

    float maxSpeed = MOTOR_MAX_SPEED - motorSpeeds[motor - 1];

    speed = constrain(speed,-maxSpeed,maxSpeed);

    MotoronI2C* controller = getMotoronTurn(motor);

    controller->setSpeedNow(getMotorChannelTurn(motor),speed);
}


// --------------------------------------------------
// DISTANZ MESSEN 
// ------------------------------------------------

float distance=0;

// --------------------------------------------------
// INITIALISIERUNG
// --------------------------------------------------

void motor_begin()
{
    Wire2.begin();
    Wire2.setClock(400000);

    motoron1.setBus(&Wire2);
    motoron2.setBus(&Wire2);
    motoron3.setBus(&Wire2);

    motoron1.setAddress(16);
    motoron2.setAddress(17);
    motoron3.setAddress(18);

    motoron1.reinitialize();
    motoron2.reinitialize();
    motoron3.reinitialize();

    motoron1.setCommandTimeoutMilliseconds(5000);
    motoron2.setCommandTimeoutMilliseconds(5000);
    motoron3.setCommandTimeoutMilliseconds(5000);

    // -------------------------------------------------
    //ENCODER
    // -------------------------------------------------

    noInterrupts();

    for (uint8_t i = 0; i < 6; i++)
    {
        encoderCounts[i] = 0;
    }

    interrupts();

    for (uint8_t i = 0; i < 6; i++)
    {
        pinMode(ENCODER_PINS_A[i], INPUT_PULLUP);
        pinMode(ENCODER_PINS_B[i], INPUT_PULLUP);
    }

    attachInterrupt(digitalPinToInterrupt(ENCODER_PINS_A[0]),encoder1ISR,CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PINS_A[1]),encoder2ISR,CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PINS_A[2]),encoder3ISR,CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PINS_A[3]),encoder4ISR,CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PINS_A[4]),encoder5ISR,CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PINS_A[5]),encoder6ISR,CHANGE);

    // --------------------------------------------------
    // STOPP PROGRAMME
    // --------------------------------------------------

    motor_stopAll();

    // --------------------------------------------------
    // WINKEL ANSTEUERN
    // --------------------------------------------------



    // --------------------------------------------------
    // FAHREN
    // --------------------------------------------------

    IMU_begin(Wire2);

    // --------------------------------------------------
    // LINIENFOLGER 
    // --------------------------------------------------

    IR_begin();

    // --------------------------------------------------
    // DREHEN 
    // --------------------------------------------------



    // --------------------------------------------------
    // DISTANZ MESSEN 
    // --------------------------------------------------
    

}

// --------------------------------------------------
// ENCODER
// --------------------------------------------------

long motor_encoder(uint8_t motor)
{
    if (motor < 1 || motor > 6)
    {
        return 0;
    }

    noInterrupts();
    int32_t count = encoderCounts[motor - 1];
    interrupts();

    return count;
}

void motor_encoderReset(uint8_t motor)
{
    if (motor < 1 || motor > 6)
    {
        return;
    }

    noInterrupts();
    encoderCounts[motor - 1] = 0;
    interrupts();
}

// --------------------------------------------------
// STOPP PROGRAMME
// --------------------------------------------------

void motor_stop(uint8_t motor)
{
    if (motor < 1 || motor > 6)
    {
        return;
    }

    if (motor == 1 || motor == 2)
    {
        MotoronI2C* controller = getMotoronDriveIMU(motor);

        controller->setSpeedNow(getMotorChannelDriveIMU(motor),0);
        return;
    }

    if (motor >= 3 && motor <= 6)
    {
        MotoronI2C* controller = getMotoronAngle(motor);

        controller->setSpeedNow(getMotorChannelAngle(motor),0);
        return;
    }
}

void motor_stopAll()
{
    for (uint8_t motor = 1; motor <= 6; motor++)
    {
        motor_stop(motor);
    }
}

// --------------------------------------------------
// WINKEL ANSTEUERN
// --------------------------------------------------

void motor_speed_angel(uint8_t motor, int speed)
{
    if (motor < 3 || motor > 6)
    {
        return;
    }

    motorSpeeds[motor - 1] = constrain(speed,-MOTOR_MAX_SPEED,MOTOR_MAX_SPEED);
}

void motor_angle(uint8_t motor, float winkel)
{
    if (motor < 3 || motor > 6 || winkel == 0.0f)
    {
        return;
    }

    int speed = abs(motorSpeeds[motor - 1]);

    if (speed == 0)
    {
        motorSpeeds[motor - 1] = MOTOR_DEFAULT_SPEED;
    }
    
    speed = abs(motorSpeeds[motor - 1]);

    float targetCounts =
        fabsf(winkel) *
        ANGLE_COUNTS_PER_REVOLUTION / 360.0f;

    float targetSigned = (winkel > 0.0f)
             ? targetCounts
             : -targetCounts;

    motor_encoderReset(motor);

    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    if (motor==3)
    {
        kp = 0.0f;
        ki = 0.0f;
        kd = 0.0f;
    }
    
    if (motor==4)
    {
        kp = 0.0f;
        ki = 0.0f;
        kd = 0.0f;
    }
    if (motor==5)
    {
        kp = 0.0f;
        ki = 0.0f;
        kd = 0.0f;
    }
    if (motor==6)
    {
        kp = 0.0f;
        ki = 0.0f;
        kd = 0.0f;
    }

    float target = targetSigned;
    float pos = 0.0f;

    float ecurr = 0.0f;
    float eprev = 0.0f;
    
    uint32_t tcurr = 0;
    uint32_t tprev = millis();

    float integral = 0.0f;
    float derivative = 0.0f;

    float u = 0.0f;
    float p = 0.0f;
    float i = 0.0f;
    float d = 0.0f;

    while (fabsf(target - (float)motor_encoder(motor)) > 1.0f)
    {          
        pos = motor_encoder(motor);

        ecurr = target - pos; 

        tcurr = millis();

        float dt = (tcurr - tprev) / 1000.0f;

        if (dt < 0.001f)
        {
            dt = 0.001f;
        }

        p = kp * ecurr;

        if (fabsf(ecurr) < 30.0f)
        {
            if (u > -speed && u < speed)
                {
                integral += ecurr * dt;
                }
            i = ki * integral;
        }
        else
        {
            integral = 0.0f;
            i = 0.0f;
        }

        derivative = (ecurr - eprev) / dt;
        d = kd * derivative;

        u = p + i + d;
    
        motor_angle_run(motor, u);

        eprev = ecurr;
        tprev = tcurr;

        delay(10);
    }

    motor_stop(motor);
}

// --------------------------------------------------
// FAHREN
// --------------------------------------------------

void motor_speed_drive(int speed)
{
    motorSpeeds[0] = constrain(speed,-MOTOR_MAX_SPEED,MOTOR_MAX_SPEED);
    motorSpeeds[1] = constrain(speed,-MOTOR_MAX_SPEED,MOTOR_MAX_SPEED);
}
 
void motor_drive(float mm)
{
    if (mm == 0.0f)
    {
        return;
    }

    int speed = abs(motorSpeeds[0]);

    if (speed == 0)
    {
        speed = MOTOR_DEFAULT_SPEED;
    }

    float wheelCircumference = PI * WHEEL_DIAMETER_MM;

    float targetCounts =
        fabsf(mm) / wheelCircumference
        * DRIVE_COUNTS_PER_REVOLUTION;

    motor_encoderReset(1);
    motor_encoderReset(2); 

    IMU_getPositionReset();

    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    float target = 0.0f;
    float pos = 0.0f;

    float ecurr = 0.0f;
    float eprev = 0.0f;

    uint32_t tcurr = 0;
    uint32_t tprev = millis();

    float integral = 0.0f;
    float derivative = 0.0f;
 
    float u = 0.0f;
    float p = 0.0f;
    float i = 0.0f;
    float d = 0.0f;

    while (true)
    {
        float leftCounts =
            fabsf((float)motor_encoder(1));

        float rightCounts =
            fabsf((float)motor_encoder(2));

        float averageCounts =
            (leftCounts + rightCounts) / 2.0f;

        if (fabsf(targetCounts - averageCounts) <= 1.0f)
        {
            break;
        } 

        pos = IMU_getPosition();

        ecurr = target - pos; 

        tcurr = millis();

        float dt = (tcurr - tprev) / 1000.0f;

        if (dt < 0.001f)
        {
            dt = 0.001f;
        }

        p = kp * ecurr;

        if (fabsf(ecurr) < 30.0f)
        {
            if (u > -speed && u < speed)
                {
                integral += ecurr * dt;
                }
            i = ki * integral;
        }
        else
        {
            integral = 0.0f;
            i = 0.0f;
        }

        derivative = (ecurr - eprev) / dt;
        d = kd * derivative;

        u = p + i + d;

        motor_drive_run_IMU(u, mm);

        eprev = ecurr;
        tprev = tcurr;

        delay(10);
    }

    motor_stop(1);
    motor_stop(2);
}

// --------------------------------------------------
// LINIENFOLGER 
// --------------------------------------------------

void motor_drive_ir(float mm, int linePosition)
{
    if (mm == 0.0f)
    {
        return;
    }

    int speed = abs(motorSpeeds[0]);

    if (speed == 0)
    {
        speed = MOTOR_DEFAULT_SPEED;
    }

    float wheelCircumference = PI * WHEEL_DIAMETER_MM;

    float targetCounts =
        fabsf(mm) / wheelCircumference
        * DRIVE_COUNTS_PER_REVOLUTION;

    motor_encoderReset(1);
    motor_encoderReset(2); 

    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    float target = linePosition;
    float pos = 0.0f;

    float ecurr = 0.0f;
    float eprev = 0.0f;

    uint32_t tcurr = 0;
    uint32_t tprev = millis();

    float integral = 0.0f;
    float derivative = 0.0f;
 
    float u = 0.0f;
    float p = 0.0f;
    float i = 0.0f;
    float d = 0.0f;

    while (true)
    {
        float leftCounts =
            fabsf((float)motor_encoder(1));

        float rightCounts =
            fabsf((float)motor_encoder(2));

        float averageCounts =
            (leftCounts + rightCounts) / 2.0f;

        if (fabsf(targetCounts - averageCounts) <= 1.0f)
        {
            break;
        }

        pos = IR_getPosition();

        ecurr = target - pos; 

        tcurr = millis();

        float dt = (tcurr - tprev) / 1000.0f;

        if (dt < 0.001f)
        {
            dt = 0.001f;
        }

        p = kp * ecurr;

        if (fabsf(ecurr) < 30.0f)
        {
            if (u > -speed && u < speed)
                {
                integral += ecurr * dt;
                }
            i = ki * integral;
        }
        else
        {
            integral = 0.0f;
            i = 0.0f;
        }

        derivative = (ecurr - eprev) / dt;
        d = kd * derivative;

        u = p + i + d;

        motor_drive_run_IR(u,mm);

        eprev = ecurr;
        tprev = tcurr;

        delay(10);
    }

    motor_stop(1);
    motor_stop(2);
}

// --------------------------------------------------
// DREHEN 
// --------------------------------------------------

void motor_turn(float winkel)
{
    if (winkel == 0.0f)
    {
        return;
    }

    int speed = abs(motorSpeeds[0]);

    if (speed == 0)
    {
        speed = MOTOR_DEFAULT_SPEED;
    }

    motor_encoderReset(1);
    motor_encoderReset(2); 

    IMU_getPositionReset();

    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    float target = 0.0f;
    float pos = 0.0f;

    float ecurr = 0.0f;
    float eprev = 0.0f;

    uint32_t tcurr = 0;
    uint32_t tprev = millis();

    float integral = 0.0f;
    float derivative = 0.0f;
 
    float u = 0.0f;
    float p = 0.0f;
    float i = 0.0f;
    float d = 0.0f;

    while (fabsf(target - IMU_getPosition()) > 1.0f)
    {
        pos = IMU_getPosition();

        ecurr = target - pos; 

        tcurr = millis();

        float dt = (tcurr - tprev) / 1000.0f;

        if (dt < 0.001f)
        {
            dt = 0.001f;
        }

        p = kp * ecurr;

        if (fabsf(ecurr) < 30.0f)
        {
            if (u > -speed && u < speed)
                {
                integral += ecurr * dt;
                }
            i = ki * integral;
        }
        else
        {
            integral = 0.0f;
            i = 0.0f;
        }

        derivative = (ecurr - eprev) / dt;
        d = kd * derivative;

        u = p + i + d;

        motor_turn_run(u, winkel);

        eprev = ecurr;
        tprev = tcurr;

        delay(10);
    }

    motor_stop(1);
    motor_stop(2);
}

void motor_turn_one(uint8_t motor, float winkel)
{
    if (motor < 1 || motor > 2 || winkel == 0.0f)
    {
        return;
    }

    int speed = abs(motorSpeeds[motor - 1]);

    if (speed == 0)
    {
        motorSpeeds[motor - 1] = MOTOR_DEFAULT_SPEED;
    }

    speed = abs(motorSpeeds[motor - 1]);

    motor_encoderReset(motor);

    IMU_getPositionReset();

    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    float target = winkel;
    float pos = 0.0f;

    float ecurr = 0.0f;
    float eprev = 0.0f;

    uint32_t tcurr = 0;
    uint32_t tprev = millis();

    float integral = 0.0f;
    float derivative = 0.0f;
 
    float u = 0.0f;
    float p = 0.0f;
    float i = 0.0f;
    float d = 0.0f;

    while (fabsf(target - IMU_getPosition()) > 1.0f)
    {
        pos = IMU_getPosition();

        ecurr = target - pos; 

        tcurr = millis();

        float dt = (tcurr - tprev) / 1000.0f;

        if (dt < 0.001f)
        {
            dt = 0.001f;
        }

        p = kp * ecurr;

        if (fabsf(ecurr) < 30.0f)
        {
            if (u > -speed && u < speed)
                {
                integral += ecurr * dt;
                }
            i = ki * integral;
        }

        else
        {
            integral = 0.0f;
            i = 0.0f;
        }

        derivative = (ecurr - eprev) / dt;
        d = kd * derivative;

        u = p + i + d;

        motor_turn_one_run(u, motor, winkel);

        eprev = ecurr;
        tprev = tcurr;

        delay(10);
    }

    motor_stop(motor);
}

// --------------------------------------------------
// DISTANZ MESSEN 
// --------------------------------------------------

float motor_distance()
{
    float leftCounts = fabsf((float)motor_encoder(1));

    float rightCounts = fabsf((float)motor_encoder(2));

    float averageCounts = (leftCounts + rightCounts) / 2.0f;

    float distance = distance + averageCounts;

    return distance;
}

void motor_distanceReset()
{
    distance=0;
}


