#include <Arduino.h>

#define RE 5
#define RM 4

#define BE 6
#define BM 7

class Motor {
public:
    int speedPin;
    int dirPin;
    
    Motor(int p_speedPin, int p_dirPin) : speedPin(p_speedPin), dirPin(p_dirPin) {
        pinMode(speedPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
    }
    
    void run(int speed) {
        
        speed %= 101;
        
        if(speed < 0) {
            digitalWrite(dirPin, 0);
            speed = -speed;
        } else {
            digitalWrite(dirPin, 1);
        }
        
        analogWrite(speedPin, abs(speed * 2.55));
    }
};

Motor R(RE, RM), B(BE, BM);

void setup() {
}

void loop() {
    B.run(-200);
}