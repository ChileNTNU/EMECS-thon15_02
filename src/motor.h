/*
 * motor.h
 *
 *  Created on: 09/05/2015
 *      Author: Emilio García
 */

#ifndef MOTOR_H_
#define MOTOR_H_

int motor_init(void);

void Move_Right(void);
void Move_Left(void);
void Move_Forward(void);
void Move_Backward(void);
void Stop_Robot(void);


#endif /* MOTOR_H_ */
