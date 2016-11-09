/*
 * SharedMotorControl.c
 *
 *  Created on: Jun 10, 2016
 *      Author: harlan.howe
 */
#include "API.h"
#include "main.h"
#include "SharedMotorControl.h"

// These variables are used to determine which direction is "forward" for this motor in code.
const int DIRECTION_MODIFIERS[] = {PORT_ORIENTATION_NORMAL,  //0 - not actually on cortex. do not use.
									PORT_ORIENTATION_NORMAL, //1
									PORT_ORIENTATION_NORMAL, //2
									PORT_ORIENTATION_REVERSED, //3
									PORT_ORIENTATION_NORMAL, //4
									PORT_ORIENTATION_REVERSED, //5
									PORT_ORIENTATION_REVERSED, //6
									PORT_ORIENTATION_NORMAL, //7
									PORT_ORIENTATION_NORMAL, //8
									PORT_ORIENTATION_NORMAL, //9
									PORT_ORIENTATION_NORMAL}; //10

/**
 *  turns on the given motor at the current power level - just like motorSet, but incorporates
 *  DIRECTION_MODIFIERS so we can assume positive is always forward.
 */
void K_setMotor(int whichPort, int power)
{
	motorSet(whichPort, power*DIRECTION_MODIFIERS[whichPort]);
}

/*
 * determines the current setting for thie given motor, -128 <-> + 128. Based on the
 * DIRECTION_MODIFIERS, so it is compatible with K_setMotor.
 */
int K_getMotor(int whichPort)
{
	return motorGet(whichPort)*DIRECTION_MODIFIERS[whichPort];
}

/**
 * sets the motor to power level 1 or -1, so the motor is free to rotate (as opposed to 0,
 * which puts on the brakes). If the motor is already set to zero, it stays at zero.
 */
void K_floatMotor(int whichPort)
{
	int current_level = motorGet(whichPort);
	if (current_level == 0)
		return;
	if (current_level < 0)
		motorSet(whichPort,-1);
	else
		motorSet(whichPort, 1);
}

/**
 * stops the motor - puts the brakes on it. Probably not a good idea if you are at top speed
 * and wish to slow down.
 */
void K_stopMotor(int whichPort)
{
	motorSet(whichPort,0);
}
