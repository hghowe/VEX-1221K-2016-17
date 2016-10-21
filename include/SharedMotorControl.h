/*
 * SharedMotorControl.h
 *
 *  Created on: Jun 10, 2016
 *      Author: harlan.howe
 */

#ifndef SHAREDMOTORCONTROL_H_
#define SHAREDMOTORCONTROL_H_

#define PORT_MOTOR_BACK_LEFT 2
#define PORT_MOTOR_BACK_RIGHT 3
#define PORT_MOTOR_FRONT_LEFT 4
#define PORT_MOTOR_FRONT_RIGHT 5

#define PORT_MOTOR_LAUNCH_LEFT 6
#define PORT_MOTOR_LAUNCH_RIGHT 7

#define PORT_MOTOR_INTAKE_JAWS 8
#define PORT_MOTOR_INTAKE_LIFT_LEFT 9
#define PORT_MOTOR_INTAKE_LIFT_RIGHT 10

#define PORT_ORIENTATION_NORMAL 1
#define PORT_ORIENTATION_REVERSED -1

#define PORT_ENCODER_FL 3
#define PORT_ENCODER_FR 2
#define PORT_ENCODER_BL 1
#define PORT_ENCODER_BR 0

bool encoderReadingFL;
bool encoderReadingFR;
bool encoderReadingBL;
bool encoderReadingBR;

int encoderFL;
int encoderFR;
int encoderBL;
int encoderBR;

/**
 *  turns on the given motor at the current power level - just like motorSet, but incorporates
 *  MOTOR_DIRECTION so we can assume positive is always forward.
 */
void K_setMotor(int whichPort, int power);

/*
 * determines the current setting for thie given motor, -128 <-> + 128. Based on the
 * DIRECTION_MODIFIERS, so it is compatible with K_setMotor.
 */
int K_getMotor(int whichPort);

/**
 * sets the motor to power level 1 or -1, so the motor is free to rotate (as opposed to 0,
 * which puts on the brakes). If the motor is already set to zero, it stays at zero.
 */
void K_floatMotor(int whichPort);

#endif /* SHAREDMOTORCONTROL_H_ */
