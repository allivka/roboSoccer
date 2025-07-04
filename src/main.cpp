
#include <Arduino.h>
#include <Wire.h>
#include <NewPing.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "HTInfraredSeeker.h"

MPU6050 mpu;

#define MAX_SONAR_DIST 200

// NewPing sonarR(19, 18, MAX_SONAR_DIST);

#define PRK 0.7
#define DRK 0.8

int const INTERRUPT_PIN = 2;

bool DMPReady = false;
uint8_t MPUIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool MPUInterrupt = false;
void DMPDataReady() {
  MPUInterrupt = true;
}

#define LE 7
#define LM 6

#define RE 8
#define RM 9

#define BE 11
#define BM 10

#define ON_PIN A5

#define CRK 2.83

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
    int pin1;
    int pin2;
    
    Motor(int p_pin1, int p_pin2) : pin1(p_pin1), pin2(p_pin2) {}
    
    void initPins() {
        pinMode(pin1, OUTPUT);
        pinMode(pin2, OUTPUT);
    }
    
    void run(int speed) {
        
        if(speed < -100) speed = -100;
        if(speed > 100) speed = 100;
        
        
        int speed1 = abs(speed) * (speed >= 0) * 2.55;
        int speed2 = abs(speed) * (speed <= 0) * 2.55;
        
        if(speed == 0) {
            speed1 = 255;
            speed2 = 255;
        }
        
        analogWrite(pin1, speed1);
        analogWrite(pin2, speed2);
        
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
        (speed * sin((60 - alpha) / 180.0 * 3.14) + w),
        (speed * sin((180 - alpha) / 180.0 * 3.14) + w)
    );
}

class Robot {
public:
    Motor L;
    Motor R;
    Motor B;
    
    int head = 0;
    
    InfraredSeeker IRSeeker;
    
    Robot(const Motor& l, const Motor& r, const Motor& b) : L(l), R(r), B(b)  {
        L.initPins();
        R.initPins();
        B.initPins();
        // IRSeeker.Initialize();
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
    
    void play(int speed) {
        updateYaw();
        
        InfraredResult result = IRSeeker.ReadAC();
        int angle = result.Direction * 30 - 150;
        
        // int distR = sonarR.ping_cm();
        
        // int alpha = correctRange(angle + sign(angle) * correctRange(CRK * result.Strength));
        int alpha = correctRange(correctRange(yaw + angle) + (55 * sign(angle)));

        
        if(result.Direction == 0) {
            this->runBalance(head, 105, speed);
            return;
        }
        
        // if(angle == 0 && result.Strength >= 170) {
            
        //     static int errold = 0;
            
        //     int err = (distR - 50);
        //     int u = err * 1.0 + (err - errold) * 0.5;
        //     errold = err;
        //     this->runBalance(head, u, speed);
        //     return;
        // }
        
        this->runBalance(head, alpha, speed);
    }
    
};

Robot robot(
    Motor(LE, LM),
    Motor(RE, RM),
    Motor(BE, BM)
);

void setup() {
    
    pinMode(ON_PIN, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(A14, OUTPUT);
    digitalWrite(A14, 1);
    delay(200);
    
    Wire.begin();
    Wire.setClock(400000);
    
    Serial.begin(9600);
    
    while(!digitalRead(ON_PIN));
    while(digitalRead(ON_PIN));
    
    Serial.println(F("Initializing I2C devices..."));
    
    initMPU();
    
    digitalWrite(LED_BUILTIN, 1);
    
    while(!digitalRead(ON_PIN));
    while(digitalRead(ON_PIN));
    
}

bool isActive = true;

void loop() {
    
    if(digitalRead(ON_PIN)) {
        isActive = !isActive;
        while(digitalRead(ON_PIN));
    }
    
    if(!isActive) {
        robot.stop();
        return;
    }
    
    if(!DMPReady) return;
    
    robot.play(50);
    Serial.println(yaw);
    // Serial.println(sonarR.ping_cm());
    // delay(5);
    
}
