/*
* @file motor.h
* @author Alexis Noel
* @version 1.0
* @date 22 mars 2024
* @brief Déclaration des fonctions contrôlant les deux moteur à l'aide du timer4.
*/

#ifndef MOTOR_H_
#define MOTOR_H_

#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>

#define LEFT_MOTOR	0
#define RIGHT_MOTOR	1

/*
* @brief Fonction servant à initialiser un moteur.
* @param uint8_t motor : soit le moteur gauche ou droit
* @return void
*/
void motorInit(uint8_t motor);

/*
* @brief Fonction servant à contrôler un moteur.
* @param uint8_t motor : moteur à contrôler (moteur gauche ou droit), uint8_t speed : vitesse du moteur, uint8_t direction : direction du moteur (avant ou arrière).
* @return void
*/
void motorControl(uint8_t motor ,uint8_t speed, uint8_t direction);

#endif /* MOTOR_H_ */