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
int x_input=0, y_input=0, angle_input=0;
int launch_direction;

//the overall template here is a state machine on currentAutoTask
//individual autoTask() functions will set currentAutoTask, throwing it to the next task

int currentAutoTask=1; //Will change 0,1,2,etc. based on the task that should be running

float lastPosition = 0;
float anglePosition = 0;

//These scale factors help adjust our different wheels.  This may
float FLScale = 3500;
float FRScale = -3500;
float BLScale = -3500;
float BRScale = 3500;
float FLAScale = 3500;
float FRAScale = 3500;
float BLAScale = -3500;//signs are differnt for turning than for distance.
float BRAScale = -3500;

void autonomous() {
	//lastPosition = LPos();
	currentAutoTask = 1;
	taskRunLoop(runLoop,1);

}

void runLoop(int state) {
	autoCheckSensors();
	autoUpdateScreen();
	autoDelegateTask();
	autoProcessMotors();
}

//float LPos() {
//	return (encoderFL/FLScale + encoderFR/FRScale + encoderBL/BLScale + encoderBR/BRScale)/4;
//}
//
//
//float APos()
//{
//	return (encoderFL/FLAScale + encoderFR/FRAScale + encoderBL/BLAScale + encoderBR/BRAScale)/4;
//
//}
void autoCheckSensors() {

	bool encoderReadingFL=false;
	bool encoderReadingFR=false;
	bool encoderReadingBL=false;
	bool encoderReadingBR=false;

	// read gyro value
	gyroVal = gyroGet(MyGyro);

	// read the Jaw Limit Switches
		jaw_open = digitalRead(PORT_INPUT_JAW_OPEN);
		jaw_close = digitalRead(PORT_INPUT_JAW_CLOSE);

		encoderReadingFL = imeGet(PORT_ENCODER_FL, &encoderFL);
		encoderReadingFR = imeGet(PORT_ENCODER_FR, &encoderFR);
		encoderReadingBL = imeGet(PORT_ENCODER_BL, &encoderBL);
		encoderReadingBR = imeGet(PORT_ENCODER_BR, &encoderBR);

		if (!encoderReadingFL || !encoderReadingFR || !encoderReadingBL || !encoderReadingBR)
		{
			currentAutoTask=0;//All stop encoder fail
			lcdPrint(uart1,1,"All Stop Encoder Fail");

		}
}

void autoUpdateScreen() {

	lcdPrint(uart1,1,"x:%d y:%d a:%d",x_input,y_input,angle_input); //driving data
	lcdPrint(uart1, 1, "Task Num: %d", currentAutoTask); //current task number
	lcdPrint(uart1, 2, "Direction: %d", intake_jaw_direction);
}

void autoDelegateTask() {
//the individual tasks will handle incrementing the currentAutotask variable
	switch (currentAutoTask) {
	/*
	case 1:
			autoTask1(); //move the robot forward
			break;
	case 2: autoTask2();//stop the robot
			break;
	case 3: //autoTask3();//turn right
			break;
	default: lcdPrint(uart1, 2, "Task Failed"); //fail msg
			 break;
	}
	*/
	case 1:
		//autoTurnToHeading(90);
		autoTask1();
		break;
	case 2:
		autoTask2();
		break;
	case 3:
		autoTask3();
		break;
	default:
		lcdPrint(uart1, 2, "Task Failed"); //fail msg
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

	K_setMotor(PORT_MOTOR_INTAKE_LEFT, 127*launch_direction);
	K_setMotor(PORT_MOTOR_INTAKE_RIGHT, 127*launch_direction);
	//K_setMotor(PORT_MOTOR_INTAKE_JAWS, 127*intake_jaw_direction);

}

void autoMoveFwd() {
	// x_input and y_input are being set directly here (since the autoProcessMotors code came
	// from opcontrol.c) instead of taking their values from the joystick.

	K_setMotor(PORT_MOTOR_FRONT_LEFT,127);
	K_setMotor(PORT_MOTOR_FRONT_RIGHT,127);
	K_setMotor(PORT_MOTOR_BACK_LEFT,127);
	K_setMotor(PORT_MOTOR_BACK_RIGHT,127);


}

void autoMoveBack() {
	// x_input and y_input are being set directly here (since the autoProcessMotors code came
	// from opcontrol.c) instead of taking their values from the joystick.

	x_input = 0;
	y_input = -110;


}

void autoTurnToHeading(float desiredHeading) {
	/*// OLD
	float startGyroVal = gyroVal;

	if(!(gyroVal == startGyroVal + 90)) {
		angle_input = -127;
	} else {
		angle_input = 0;
	}
	*/

	float currentDesiredChange = findDesiredChangeInDegrees(desiredHeading);

	if((currentDesiredChange > 5)) { // within 5 degrees is "good enough"
		angle_input = -127; // turn left
	} else if((currentDesiredChange < -5)) {
		angle_input = 127; // turn right
	} else {
		angle_input = 0; // stop
	}

	lcdPrint(uart1,1,"Heading: %f3.2", desiredHeading);
	lcdPrint(uart1,2,"GryoVal: %d", gyroVal);
}

float convertRawGyroValToDegrees() {
	float currentGyroVal = gyroVal; // get current gyro val

	if (currentGyroVal > 0) {
		while(currentGyroVal >= 360) {
				currentGyroVal -= 360.0;
			}
	} else {
		while(currentGyroVal < 0) {
				currentGyroVal += 360.0;
			}
	}

	return currentGyroVal;
}

float findDesiredChangeInDegrees(float desiredHeading) {
	float currentGyroDegrees = convertRawGyroValToDegrees();
	float desiredChange = desiredHeading - currentGyroDegrees;

	if(desiredChange > 180) {
		while(desiredChange > 180) {
			desiredChange -= 180.0;
		}
	} else if(desiredChange < -180) {
		while(desiredChange <= -180) {
			desiredChange += 180.0;
		}
	}

	return desiredChange;
}

void autoStop() {
	// x_input and y_input are being set directly here (since the autoProcessMotors code came
	// from opcontrol.c) instead of taking their values from the joystick.

	K_setMotor(PORT_MOTOR_FRONT_LEFT,0);
	K_setMotor(PORT_MOTOR_FRONT_RIGHT,0);
	K_setMotor(PORT_MOTOR_BACK_LEFT,0);
	K_setMotor(PORT_MOTOR_BACK_RIGHT,0);


}


//autoTask functions, must set controls and wait until a given event.
//when the event is triggered, these must increment currentAutotask to move to the next event
//Do not put any large loops in autotasks, these should just set controls and then drop back to
//the autoloop
void autoTask1() {
	autoMoveFwd();
	delay(1600);
	autoStop();
	delay(1000);

	K_setMotor(PORT_MOTOR_FLIPPER_LEFT, 127);
	K_setMotor(PORT_MOTOR_FLIPPER_RIGHT, 127);
	K_setMotor(PORT_MOTOR_INTAKE_LEFT, 127);
	K_setMotor(PORT_MOTOR_INTAKE_RIGHT, 127);
	delay(1000);

	K_setMotor(PORT_MOTOR_FLIPPER_LEFT, 0);
	K_setMotor(PORT_MOTOR_FLIPPER_RIGHT, 0);
	K_setMotor(PORT_MOTOR_INTAKE_LEFT, 0);
	K_setMotor(PORT_MOTOR_INTAKE_RIGHT, 0);
	delay(8000);

	//currentAutoTask += 1;

//	intake_lift_direction = 1;
//	delay(250);
//	intake_lift_direction = 0;
//
//	launch_direction = 1;
//	delay(250);
//	launch_direction = 0;

//	if ((LPos() - lastPosition) > 2000) {currentAutoTask++;}
//	lcdPrint(uart1,1,"%3.2f", (LPos()));
}

void autoTask2() {
	autoStop();
}

void autoTask3() {

}
