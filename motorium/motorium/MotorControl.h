#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

// control commands
#define FORWARD		0x61
#define REVERSE		0x62
#define STOP		0x63
#define TURNRIGHT	0x64
#define TURNLEFT	0x65
#define TURNRIGHTTO	0x66
#define TURNLEFTTO	0x67

void drive_init(void);

// simple directional control

void forward(void);
void reverse(void);
void turnLeft(void);
void turnRight(void);
void allStop(void);

void turnLeftTo(int16_t targetBearing);
void turnRightTo(int16_t targetBearing);

void turnTo(int16_t targetBearing);

void turnRightBy(int16_t degrees);
void turnLeftBy(int16_t degrees);

#endif /* MOTORCONTROL_H_ */