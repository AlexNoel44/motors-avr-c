/*
* @file motor.c
* @author Alexis Noel
* @version 1.0
* @date 22 mars 2024
* @brief Definition des fonctions contrôlant les deux moteur à l'aide du timer4.
*/

#include "motor.h"

#define NB_MOTOR					2

#define LEFT_MOTOR_FORWARD_PIN		(1<<6)
#define LEFT_MOTOR_BACKWARD_PIN		(1<<5)
#define LEFT_MOTOR_DEACTIVATION		(DDRB & ~((1<<5)|(1<<6)))

#define RIGHT_MOTOR_FORWARD_PIN		(1<<7)
#define RIGHT_MOTOR_BACKWARD_PIN	(1<<6)
#define RIGHT_MOTOR_DEACTIVATION	(DDRD & ~((1<<6)|(1<<7)))

#define FORWARD_DIRECTION			1
#define BACKWARD_DIRECTION			0

typedef struct
{
	uint8_t speed;
	uint8_t direction;
}Motor;

Motor motors[NB_MOTOR];

/////////////////////////////////////////////////////////////////////////////////////////////////"FONCTIONS PRIVÉES//////////////////////////////////////////////////////////////////////////////////////////////

/*
* @brief Fonction servant a donner une direction l'instance d'un des moteurs.
* @param uint8_t motor : moteur gauche ou droit, uint8_t direction : direction du moteur.
* @return void
*/
void _setMotorDirection(uint8_t motor, uint8_t direction)
{
	motors[motor].direction=direction;
}

/*
* @brief Fonction servant a retourner la direction l'instance d'un des moteurs.
* @param uint8_t motor : moteur gauche ou droit.
* @return uint8_t motors[motor].direction : direction attribuée à l'instance du moteur.
*/
uint8_t _getMotorDirection(uint8_t motor)
{	
	return motors[motor].direction;
}

/***************************************************************************************************************************************************************************************************************/

/*
* @brief Fonction servant a donner une vitesse l'instance d'un des moteurs.
* @param uint8_t motor : moteur gauche ou droit, uint8_t speed : vitesse du moteur.
* @return void
*/
void _setMotorSpeed(uint8_t motor, uint8_t speed)
{
	((speed < 10) ? (speed=speed-1) : (speed=0));
	
	if(motors[motor].direction==FORWARD_DIRECTION)
		motors[motor].speed=speed;
	else if(motors[motor].direction==BACKWARD_DIRECTION)
		motors[motor].speed=8-(speed);
}

/*
* @brief Fonction servant a retourner la vitesse l'instance d'un des moteurs.
* @param uint8_t motor : moteur gauche ou droit.
* @return uint8_t motors[motor].vitesse : vitesse attribuée à l'instance du moteur.
*/
uint8_t _getMotorSpeed(uint8_t motor)
{
	return motors[motor].speed;
}

////////////////////////////////////////////////////////////////////////////////"FONCTIONS PUBLIQUES//////////////////////////////////////////////////////////////////////////////////////////////

void motorInit(uint8_t motor)
{
	
	//DDRB |= (1<<5) | (1<<6); //OC4B/ et OC4B
	//DDRD |= (1<<6) | (1<<7); //OC4D/ et OC4D
		
	TCCR4A |= (1<<COM4B0) | (1<<PWM4B) | (1<<COM4A0) | (1<<PWM4A);
	TCCR4B |= (1<<CS40) | (1<<CS42);
	TCCR4C |= (1<<COM4D0) | (1<<PWM4D);
		
	OCR4C = 10-1;
	
	sei();
	
	switch (motor)
	{
		case LEFT_MOTOR:
			_setMotorSpeed(LEFT_MOTOR, 0);
			_setMotorDirection(LEFT_MOTOR, 1);
			break;
		
		case RIGHT_MOTOR:
			_setMotorSpeed(RIGHT_MOTOR, 0);
			_setMotorDirection(RIGHT_MOTOR, 1);
			break;
	}
}

/***************************************************************************************************************************************************************************************************************/

void motorControl(uint8_t motor, uint8_t speed, uint8_t direction)
{
	
	_setMotorDirection(motor,direction);
	_setMotorSpeed(motor, speed);
	
	switch(motor)
	{
		case LEFT_MOTOR:
			//DDRB = (DDRB & ~((1<<5)|(1<<6))) | (_getMotorDirection(motor) ? (1<<6) : (1<<5));
			DDRB = LEFT_MOTOR_DEACTIVATION | (_getMotorDirection(motor) ? LEFT_MOTOR_FORWARD_PIN : LEFT_MOTOR_BACKWARD_PIN);
			OCR4B=_getMotorSpeed(motor);
			break;
			
		case RIGHT_MOTOR:
			//DDRD = (DDRD & ~((1<<6)|(1<<7))) | (_getMotorDirection(motor) ? (1<<7) : (1<<6));
			DDRD = RIGHT_MOTOR_DEACTIVATION | (_getMotorDirection(motor) ? RIGHT_MOTOR_FORWARD_PIN : RIGHT_MOTOR_BACKWARD_PIN);
			OCR4D=_getMotorSpeed(motor);
			break; 
	}
}
