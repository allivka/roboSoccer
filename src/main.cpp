
#include <Arduino.h>
#include <Wire.h>
#include <NewPing.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "HTInfraredSeeker.h"

// InfraredResult IRData = IRSeeker.ReadAC(); example of IrSeeker usage

MPU6050 mpu;

#define MAX_SONAR_DIST

NewPing sonarL(A0, A1, 200);
NewPing sonarR(9, 8, 200);

#define PRK 0.75
#define DRK 4.5

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

#define CRK 0.7

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
        digitalWrite(LED_BUILTIN, 1);
        
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
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

int sign(int n) {
    if(n < 0) return -1;
    if(n > 0) return 1;
    return 0;
}

int correctRange(int angle) {
    if(angle < -180.0) angle += 360.0;
    if(angle > 180.0) angle += -360.0;
    return angle;
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
        
        // if(speed < -255) speed = -255;
        // if(speed > 255) speed = 255;
        
        if(speed < -100) speed = -100;
        if(speed > 100) speed = 100;
        
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
    
    InfraredSeeker IRSeeker;
    
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
    
    void runBalance(int targetDir, int relDirection, int speed) {
        static float errold = 0;
        float err = targetDir - yaw;
        
        if(err < -180.0) err += 360.0;
        if(err > 180.0) err += -360.0;
        
        float u = err * PRK + (err - errold) * DRK;
        errold = err;
        u /= 1.8;
        
        this->run(countSpeeds(relDirection, speed, u));
    }
        
    void runBalanceAlter(int absDir, int speed) {
        static float errold = 0;
        float err = absDir - yaw;
        
        if(err < -180.0) err += 360.0;
        if(err > 180.0) err += -360.0; 
        
        float u = err * 1.5 + (err - errold) * 0.0;
        errold = err;
        u /= 1.8;
        
        this->run(countSpeeds(u, speed, 0));
    }
    
    void followBall() {
        updateYaw();
    
        InfraredResult result = IRSeeker.ReadAC();
        int angle = result.Direction * 30 - 150;
        
        Serial.print("angle\t");
        Serial.print(angle);
        
        if(result.Direction != 0) {
            updateYaw();
            this->runBalance(yaw + angle, 0, 40);
        } else {
            this->run(countSpeeds(0, 0, 30));
        }
    }
    
    
    void catchBall(int speed) {
        updateYaw();
        
        InfraredResult result = IRSeeker.ReadAC();
        int angle = result.Direction * 30 - 150;
        
        int distL = sonarL.ping_cm();
        int distR = sonarR.ping_cm();
        
        // int alpha = correctRange(angle + angle * CRK * result.Strength * sign(angle));
        int alpha = correctRange(yaw + angle + (70 * sign(angle)));
        
        // Serial.print("\tangle\t");
        // Serial.print(angle);
        // Serial.print("\tstrength\t");
        // Serial.print(result.Strength);
        // Serial.print("\talpha\t");
        // Serial.println(alpha);
        
        if(result.Direction == 0) {
            this->runBalance(head, 105, speed);
            return;
        }
        
        if(angle == 0 && result.Strength >= 215) {
            
            static int errold = 0;
            
            int err = (distR - distL);
            int u = err * 0.25 + (err - errold) * 0.1;
            errold = err;
            this->runBalance(head, alpha + u, speed);
            return;
        }
        
        this->runBalance(head, alpha, speed);
    }
    
};

Robot robot(
    Motor(LE, LM),
    Motor(RE, RM),
    Motor(BE, BM)
);

void switchFalling() {
    robot.stop();
    while(!digitalRead(ON_PIN)) {
        delay(1);
    }
}

void printLogs() {
    Serial.print("yaw\t");
    Serial.println(yaw);
}

void setup() {
    pinMode(ON_PIN, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    
    Wire.begin();
    Wire.setClock(400000);
    
    Serial.begin(115200);
    
    Serial.println(F("Initializing I2C devices..."));
    
    pinMode(INTERRUPT_PIN, INPUT);
    
    while(!digitalRead(ON_PIN)) delay(1);
    initMPU();
    while(digitalRead(ON_PIN)) delay(1);
    attachInterrupt(INT2, switchFalling, FALLING);
    
}

void loop() {
    if(!DMPReady) return;
    
    
    
    // Serial.print("\tLeft\t");
    // Serial.print(distL);
    // Serial.print("\tRight\t");
    // Serial.println(distR);
    
    robot.catchBall(70);
}
