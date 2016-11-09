/*
 * auto.h
 *
 *  Created on: Jul 10, 2016
 *      Author: harlan.howe
 */

#ifndef AUTO_H_
#define AUTO_H_

#define PORT_INPUT_JAW_OPEN 1 //DIGITAL PORT 1 (OUTER SENSOR)
#define PORT_INPUT_JAW_CLOSE 2 //DIGITAL PORT 2 (INNER SENSOR)

void runLoop();
void autoCheckSensors();
void autoUpdateScreen();
void autoDelegateTask();
int autoNormalizeMotorPower();
void autoProcessMotors();

float LPos();

void autoTask1();
void autoTask2();
/*
void autoTask1();
void autoTask2();
void autoTask3();
*/

#endif /* AUTO_H_ */
