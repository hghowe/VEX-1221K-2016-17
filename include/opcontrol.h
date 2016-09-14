/*
 * opcontrol.h
 *
 *  Created on: Jul 10, 2016
 *      Author: harlan.howe
 */

#ifndef OPCONTROL_H_
#define OPCONTROL_H_

#define PORT_INPUT_JAW_OPEN 1 //DIGITAL PORT 1 (OUTER SENSOR)
#define PORT_INPUT_JAW_CLOSE 2 //DIGITAL PORT 2 (INNER SENSOR)
#define PORT_INPUT_POTENTIOMETER 1 //ANALOG PORT 1

void checkSensors();
void updateScreen();
void processMotors();
int normalizeMotorPower(int power);

#endif /* OPCONTROL_H_ */
