
#include <Arduino.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

int const INTERRUPT_PIN = 2;

bool DMPReady = false;
uint8_t MPUIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

volatile bool MPUInterrupt = false;
void DMPDataReady() {
  MPUInterrupt = true;
}

#define LE 5
#define LM 4

#define RE 10
#define RM 11

#define BE 6
#define BM 7

#define BSDK 0.73

#define ON_PIN 0

void switchFalling();

void initMPU() {
    mpu.initialize();
    
    Serial.println(F("Testing MPU6050 connection..."));
    if(mpu.testConnection() == false){
        Serial.println("MPU6050 connection failed");
        while(true);
    }
    else {
        Serial.println("MPU6050 connection successful");
    }
    
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println("These are the Active offsets: ");
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
        MPUIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        DMPReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void setup() {
    pinMode(ON_PIN, INPUT);
    
    Wire.begin();
    Wire.setClock(400000);
    
    Serial.begin(115200);
    while (!Serial);
    
    Serial.println(F("Initializing I2C devices..."));
    
    pinMode(INTERRUPT_PIN, INPUT);
    
    while(!digitalRead(ON_PIN)) delay(1);
    initMPU();
    while(digitalRead(ON_PIN)) delay(1);
    // attachInterrupt(INT2, switchFalling, FALLING);
    
}

float yaw = 0.0;

void updateYaw() {
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        yaw = ypr[0] * 180 / M_PI;

    }
}

class Motor {
public:
    int speedPin;
    int dirPin;
    
    Motor() = default;
    
    Motor(int p_speedPin, int p_dirPin) : speedPin(p_speedPin), dirPin(p_dirPin) {}
    
    void initPins() {
        pinMode(speedPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
    }
    
    void run(int speed) {
        
        if(speed < -255) speed = -255;
        if(speed > 255) speed = 255;
        
        if(speed >= 0) {
            digitalWrite(dirPin, 0);
            speed = -speed;
        } else {
            digitalWrite(dirPin, 1);
        }
        
        analogWrite(speedPin, abs(speed * 2.55));
    }
};

class RobotSpeeds {
public:
    int L = 0;
    int R = 0;
    int B = 0;
    RobotSpeeds(int l, int r, int b) : L(l), R(r), B(b) {}
};

RobotSpeeds countSpeeds(int alpha, int speed, int w) {
    speed = -speed;
    
    return RobotSpeeds(
        (speed * sin((-60 - alpha) / 180.0 * 3.14) + w),
        -(speed * sin((60 - alpha) / 180.0 * 3.14) + w),
        (speed * sin((180 - alpha) / 180.0 * 3.14) + w) * BSDK
    );
}

class Robot {
public:
    Motor L;
    Motor R;
    Motor B;
    
    int head = 0;
    
    Robot() = default;
    
    Robot(const Motor& l, const Motor& r, const Motor& b) : L(l), R(r), B(b)  {
        L.initPins();
        R.initPins();
        B.initPins();
    }
    
    void run(const RobotSpeeds& speeds) {
        L.run(speeds.L);
        R.run(speeds.R);
        B.run(speeds.B);
    }
    
    void stop() {
        L.run(0);
        R.run(0);
        B.run(0);
    }
    
};

Robot robot(
    Motor(LE, LM),
    Motor(RE, RM),
    Motor(BE, BM)
);

// void switchFalling() {
//     robot.head = yaw;
//     robot.stop();
//     while(!digitalRead(ON_PIN)) delay(1);
// }

void printLogs() {
    Serial.print("yaw\t");
    Serial.println(yaw);
}

void loop() {
    if(!DMPReady) return;
    
    updateYaw();
    
    if(!digitalRead(ON_PIN)) {
        robot.head = yaw;
        robot.stop();
    } else {
    
        float err = robot.head - yaw;
        if(err < -180.0) err += 360.0;
        if(err > 180.0) err += -360.0;
        float u = err * 1;
        
        robot.run(countSpeeds(0, 0, u));
    }
    
    printLogs();
}
