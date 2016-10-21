/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
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

#include "main.h"
#include "auto.h"
#include "SharedMotorControl.h"

/*
 * Runs the user autonomous code. This function will be started in its own task with the default
 * priority and stack size whenever the robot is enabled via the Field Management System or the
 * VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is
 * lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart
 * the task, not re-start it from where it left off.
 *
 * Code running in the autonomous task cannot access information from the VEX Joystick. However,
 * the autonomous function can be invoked from another task if a VEX Competition Switch is not
 * available, and it can access joystick information if called in this way.
 *
 * The autonomous task may exit, unlike operatorControl() which should never exit. If it does
 * so, the robot will await a switch to another mode or disable/enable cycle.
 */

int jaw_open;
int jaw_close;
int intake_jaw_direction;
int x_input, y_input, angle_input;
int launch_direction;
int currentAutoTask; //Will change 0,1,2,etc. based on the task that should be running

void autonomous() {

	currentAutoTask = 1;
	taskRunLoop(runLoop,1);

}
void runLoop(int state) {
	autoCheckSensors();
	autoUpdateScreen();
	autoDelegateTask();
	autoProcessMotors();
}

void autoCheckSensors() {

	// read the Jaw Limit Switches
		jaw_open = digitalRead(PORT_INPUT_JAW_OPEN);
		jaw_close = digitalRead(PORT_INPUT_JAW_CLOSE);

		/*
		// if we hit one of the Jaw Limit Switches, we may need to stop the jaw motor....
		if ((intake_jaw_direction == 1) && (jaw_open == 0))
			intake_jaw_direction = 0; //resets motion if outer limit switch trips
		if ((intake_jaw_direction == -1) && (jaw_close == 0))
			intake_jaw_direction = 0; //resets motion if inner limit switch trips
		*/
}

void autoUpdateScreen() {

	//lcdPrint(uart1,1,"x:%d y:%d a:%d",x_input,y_input,angle_input); //driving data
	//lcdPrint(uart1, 1, "Task Num: %d", currentAutoTask); //current task number
	//lcdPrint(uart1, 2, "Direction: %d", intake_jaw_direction);
}

void autoDelegateTask() {

	switch (currentAutoTask) {
	case 1: //autoTask1();
			break;
	case 2: //autoTask2();
			break;
	case 3: //autoTask3();
			break;
	default: lcdPrint(uart1, 2, "Task Failed"); //fail msg
			 break;
	}

}

int autoNormalizeMotorPower(int power)
{
	if (power>127)
		return 127;
	if (power<-127)
		return -127;
	if (power<10 && power>-10)
		return 0;
	return power;

}

void autoProcessMotors() {

	int RF_motor_power = autoNormalizeMotorPower(y_input - x_input - angle_input);
	int RB_motor_power = autoNormalizeMotorPower(y_input + x_input - angle_input);
	int LF_motor_power = autoNormalizeMotorPower(y_input + x_input + angle_input);
	int LB_motor_power = autoNormalizeMotorPower(y_input - x_input + angle_input);

	K_setMotor(PORT_MOTOR_FRONT_LEFT,LF_motor_power);
	K_setMotor(PORT_MOTOR_BACK_LEFT,LB_motor_power);
	K_setMotor(PORT_MOTOR_FRONT_RIGHT,RF_motor_power);
	K_setMotor(PORT_MOTOR_BACK_RIGHT,RB_motor_power);

	K_setMotor(PORT_MOTOR_LAUNCH_LEFT, 127*launch_direction);
	K_setMotor(PORT_MOTOR_LAUNCH_RIGHT, 127*launch_direction);
	K_setMotor(PORT_MOTOR_INTAKE_JAWS, 127*intake_jaw_direction);

}

void autoMoveFwd(int duration) {
	// x_input and y_input are being set directly here (since the autoProcessMotors code came
	// from opcontrol.c) instead of taking their values from the joystick.

	x_input = 0;
	y_input = 110;

	//figure out how to wait for "duration" (the variable) seconds WITHOUT freezing the runLoop()
}

void autoMoveBack(int duration) {
	// x_input and y_input are being set directly here (since the autoProcessMotors code came
	// from opcontrol.c) instead of taking their values from the joystick.

	x_input = 0;
	y_input = -110;

	//figure out how to wait for "duration" (the variable) seconds WITHOUT freezing the runLoop()
}
/*
void autoTask1() {
	if(jaw_open == 0) { //jaw_open == 0 means switch has been triggered
		intake_jaw_direction = 0;
		currentAutoTask = 2;
	}
	else {
		intake_jaw_direction = 1;
	}
}

void autoTask2() {
	if(jaw_close == 0) {
		intake_jaw_direction = 0;
		currentAutoTask = 3;
	}
	else {
		intake_jaw_direction = -1;
	}
}

void autoTask3() {
	intake_jaw_direction = 0;
}
*/
