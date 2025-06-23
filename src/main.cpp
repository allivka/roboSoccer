#include <Arduino.h>

#define LE 5
#define LM 4

#define RE 10
#define RM 11

#define BE 6
#define BM 7

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
        
        if(speed >= 0) {
            digitalWrite(dirPin, 0);
            speed = -speed;
        } else {
            digitalWrite(dirPin, 1);
        }
        
        speed %= 101;
                
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
    return RobotSpeeds(
        speed * sin(-60 - alpha) + w,
        speed * sin(60 - alpha) + w,
        speed * sin(180 - alpha) + w
    );
}

class Robot {
public:
    Motor L;
    Motor R;
    Motor B;
    
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

void setup() {
    
}

void loop() {
    
}