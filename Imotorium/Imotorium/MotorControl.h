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

void turnLeftTo(int targetBearing);
void turnRightTo(int targetBearing);

void turnTo(int targetBearing);


void turnRightBy(int degrees);






#endif /* MOTORCONTROL_H_ */