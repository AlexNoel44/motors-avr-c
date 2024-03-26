/*
* @file motor.h
* @author Alexis Noel
* @version 1.0
* @date 22 mars 2024
* @brief D�claration des fonctions contr�lant les deux moteur � l'aide du timer4.
*/

#ifndef MOTOR_H_
#define MOTOR_H_

#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>

#define LEFT_MOTOR	0
#define RIGHT_MOTOR	1

/*
* @brief Fonction servant � initialiser un moteur.
* @param uint8_t motor : soit le moteur gauche ou droit
* @return void
*/
void motorInit(uint8_t motor);

/*
* @brief Fonction servant � contr�ler un moteur.
* @param uint8_t motor : moteur � contr�ler (moteur gauche ou droit), uint8_t speed : vitesse du moteur, uint8_t direction : direction du moteur (avant ou arri�re).
* @return void
*/
void motorControl(uint8_t motor ,uint8_t speed, uint8_t direction);

#endif /* MOTOR_H_ */