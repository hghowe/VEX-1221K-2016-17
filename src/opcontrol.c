/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * Copyright (c) 2011-2014, Purdue University ACM SIG BOTS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Purdue University ACM SIG BOTS nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */
// PROJECT: MECANUM DRIVE

#include "main.h"
#include "opcontrol.h"
#include "SharedMotorControl.h"

/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */

int x_input, y_input, angle_input;
int launch_direction, intake_jaw_direction, intake_lift_direction;
int jaw_open, jaw_close, jaw_potentiometer;

void operatorControl()
{
	while (1)
	{
		checkSensors();
		updateScreen();
		processMotors();
		wait(12);
	}
}

//-----------------------------------------
//  NEW METHODS.
//   note: if you wish to add more methods to this file, you need to put declarations in main.h.
//-----------------------------------------
/**
 * Read what's going on with the joystick(s) and assign that information to variables.
 */
void checkSensors()
{
	// read the joysticks - they control the motors.
	x_input = joystickGetAnalog(1,1);
	y_input = joystickGetAnalog(1,2);
	angle_input = joystickGetAnalog(1,4);

	// read the right hand shoulder buttons on the joystick - they control the firing
			//of the launcher.
	if (joystickGetDigital(1,6,JOY_UP) && !joystickGetDigital(1,5,JOY_UP))
		launch_direction = 1;
	else if (!joystickGetDigital(1,6,JOY_UP) && joystickGetDigital(1,5,JOY_UP))
		launch_direction = -1;
	else
		launch_direction = 0;

	// read the left cluster's left/right buttons on joystick - they control jaw in/out
	if (joystickGetDigital(1,7,JOY_LEFT) && !joystickGetDigital(1,7,JOY_RIGHT))
		intake_jaw_direction = 1; //open jaw
	else if (!joystickGetDigital(1,7,JOY_LEFT) && joystickGetDigital(1,7,JOY_RIGHT))
		intake_jaw_direction = -1; //close jaw
	else
		intake_jaw_direction = 0;

	// read the left cluster's up/down buttons on joystick - they control jaw up/down
		if (joystickGetDigital(1,7,JOY_UP) && !joystickGetDigital(1,7,JOY_DOWN))
			intake_lift_direction = 1; //jaw up
		else if (!joystickGetDigital(1,7,JOY_UP) && joystickGetDigital(1,7,JOY_DOWN))
			intake_lift_direction = -1; //jaw down
		else
			intake_lift_direction = 0;

	// read the Jaw Limit Switches
	jaw_open = digitalRead(PORT_INPUT_JAW_OPEN);
	jaw_close = digitalRead(PORT_INPUT_JAW_CLOSE);
	jaw_potentiometer = analogRead(PORT_INPUT_POTENTIOMETER);

	// if we hit one of the Jaw Limit Switches, we may need to stop the jaw motor....
	if ((intake_jaw_direction == 1) && (jaw_open == 0))
		intake_jaw_direction = 0; //resets motion if outer limit switch trips
	if ((intake_jaw_direction == -1) && (jaw_close == 0))
		intake_jaw_direction = 0; //resets motion if inner limit switch trips

	//set limits for jaw potentiometer
	if ((intake_lift_direction == 1) && (jaw_potentiometer <= jaw_potentiometer_min)) {
		intake_lift_direction = 0;
	}
	if ((intake_lift_direction == -1) && (jaw_potentiometer >= jaw_potentiometer_max)) {
		intake_lift_direction = 0;
	}
}

/**
 * put relevant information up on the LCD screen for debugging, based on the variables we
 * are using.
 * Note: to use the LCD screen, in the init.c file, you have to add two lines to initializeIO():
 *   lcdInit(uart1);
 *   lcdClear(uart1);
 */
void updateScreen()
{
	//lcdPrint(uart1,1,"x:%d y:%d a:%d",x_input,y_input,angle_input); //driving data
	lcdPrint(uart1, 1, "Direction: %d", intake_jaw_direction); //intake data
	lcdPrint(uart1, 2, "Jaw Pot: %d", jaw_potentiometer); //potentiometer data


}

/**
 * Here we take the information in the variables and we apply it to the actual motor commands.
 * Motor names and their orientations are set up in SharedMotorControl.h.
 */
void processMotors()
{
	int RF_motor_power = normalizeMotorPower(y_input - x_input - angle_input);
	int RB_motor_power = normalizeMotorPower(y_input + x_input - angle_input);
	int LF_motor_power = normalizeMotorPower(y_input + x_input + angle_input);
	int LB_motor_power = normalizeMotorPower(y_input - x_input + angle_input);

	K_setMotor(PORT_MOTOR_FRONT_LEFT,LF_motor_power);
	K_setMotor(PORT_MOTOR_BACK_LEFT,LB_motor_power);
	K_setMotor(PORT_MOTOR_FRONT_RIGHT,RF_motor_power);
	K_setMotor(PORT_MOTOR_BACK_RIGHT,RB_motor_power);

	K_setMotor(PORT_MOTOR_LAUNCH_LEFT, 127*launch_direction);
	K_setMotor(PORT_MOTOR_LAUNCH_RIGHT, 127*launch_direction);
	K_setMotor(PORT_MOTOR_INTAKE_JAWS, 127*intake_jaw_direction);

	K_setMotor(PORT_MOTOR_INTAKE_LIFT_LEFT, 127*intake_lift_direction);
	K_setMotor(PORT_MOTOR_INTAKE_LIFT_RIGHT, 127*intake_lift_direction);


}

int normalizeMotorPower(int power)
{
	if (power>127)
		return 127;
	if (power<-127)
		return -127;
	if (power<10 && power>-10)
		return 0;
	return power;

}
