/*
Projet: Le nom du script
Equipe: P24
Auteurs: Mathieu Beaudoin
Description: Breve description du script
Date: Derniere date de modification
*/

/* ****************************************************************************
Inclure les librairies de functions que vous voulez utiliser
**************************************************************************** */

#include "LibRobus.h" // Essentielle pour utiliser RobUS
#include <arduino.h>
#include <stdio.h>
#include <math.h>


/* ****************************************************************************
Variables globales et defines
**************************************************************************** */
// -> defines...
// L'ensemble des fonctions y ont acces

int etape =0;

float kp = 0.0025;//0.002;
float ki = 0.00005;
float vitesse=0.55;

float vitesseAjustementDroite=vitesse;
long int distanceParcouruG =0;
const int deltat=50;
unsigned long temps=0;//deltat?
long int erreurTotal=0;

/* ****************************************************************************
Vos propres fonctions sont creees ici
**************************************************************************** */

long int longtopulse(float distance){
  // entrer distance en cm
  //info: 3200 pulse / tours, 3" = 7.62cm, 23.94cm(7.62*pi)/tours
  long int nbpulse;
  nbpulse = ceil((3200/23.94)*distance);
  return nbpulse;
}

long int angletopulse(float anglein){
  //La fonction assume que le robot a une roue fixe.
  float rayon = 18.7;// distance c/c entre les roues (cm): 
  float anglerad = anglein *(M_PI/180);
  float arclong = anglerad*rayon;
  long int nbpulsearc = longtopulse(arclong);
  return nbpulsearc;
}

double pi(long int pulseAttendu, long int pulseReel){
  int erreur = pulseAttendu - pulseReel;
  erreurTotal = erreurTotal + erreur;
  double correction = (kp*erreur)+(ki*erreurTotal);
  return correction;
}

void lignedroite(long int distance){
  while(distanceParcouruG<distance ){
    MOTOR_SetSpeed(1,0);
    MOTOR_SetSpeed(1,constrain(vitesseAjustementDroite,-1,1));
    MOTOR_SetSpeed(0,0);
    MOTOR_SetSpeed(0,vitesse);

    if(millis()-temps>=deltat){
      distanceParcouruG= distanceParcouruG+ENCODER_Read(0);
      vitesseAjustementDroite = vitesse + pi(ENCODER_Read(0),ENCODER_Read(1));
      ENCODER_Reset(0);
      ENCODER_Reset(1);
      temps=millis();
    }
  }
  MOTOR_SetSpeed(1,0);
  MOTOR_SetSpeed(0,0);
  distanceParcouruG=0;
}

/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */

void setup(){
  BoardInit();
  Serial.begin(9600);

  MOTOR_SetSpeed(0,vitesse);
  MOTOR_SetSpeed(1,vitesse);
  temps=millis();
}

/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"
 
void loop() {

  lignedroite(longtopulse(122.5));
  delay(2000);
  lignedroite(longtopulse(160));
  delay(2000); 
  
}
