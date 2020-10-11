/*
Projet: Code du défi du parcour
Equipe: P24
Auteurs: Équipe P24
Description: Séquence de déplacement du défi du parcour

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

float kp = 0.00275;//0.002;
float ki = 0.00123;
float vitesse=0.4;

float vitesseAjustementDroite=vitesse;
long int distanceParcouruG =0;
long int distanceParcouruD =0;
const int deltat=50;
unsigned long temps=0;//deltat?
long int erreurTotal=0;

float distanceroues =18.6;

int angle;
#define PI 3.1415926535897932384626433832795

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

double distancetourner (int angle) {
double dt;
dt=PI*2*longtopulse(distanceroues)*angle/360.0;
return dt;
}


double pi(long int pulseAttendu, long int pulseReel){
  int erreur = pulseAttendu - pulseReel;
  erreurTotal = erreurTotal + erreur;
  double correction = (kp*erreur)+(ki*erreurTotal);
  return correction;
}

void lignedroite(long int distance){
  while(distanceParcouruG<distance ){
    MOTOR_SetSpeed(1,constrain(vitesseAjustementDroite,-1,1));
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
  distanceParcouruD=0;
  erreurTotal = 0;
}

void tournergauche(int angle){
while(distanceParcouruD<distancetourner(angle)){
  MOTOR_SetSpeed(0,0);
  MOTOR_SetSpeed(1,vitesse);
   if(millis()-temps>=deltat){
      distanceParcouruD= distanceParcouruD+ENCODER_Read(1);
      vitesseAjustementDroite = vitesse + pi(distancetourner(angle),ENCODER_Read(1));
      ENCODER_Reset(0);
      ENCODER_Reset(1);
      temps=millis();
    }
}
MOTOR_SetSpeed(0,0);
MOTOR_SetSpeed(1,0);
distanceParcouruD=0;
distanceParcouruG=0;
erreurTotal = 0;
}

void tournerdroite(int angle){
while(distanceParcouruG<distancetourner(angle)){
  MOTOR_SetSpeed(1,0);
  MOTOR_SetSpeed(0,vitesse);
   if(millis()-temps>=deltat){
      distanceParcouruG= distanceParcouruG+ENCODER_Read(0);
      vitesseAjustementDroite = vitesse + pi(distancetourner(angle),ENCODER_Read(0));
      ENCODER_Reset(0);
      ENCODER_Reset(1);
      temps=millis();
    }
}
MOTOR_SetSpeed(0,0);
MOTOR_SetSpeed(1,0);
distanceParcouruD=0;
distanceParcouruG=0;
erreurTotal=0;
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

//Fonction pour l'aller
  lignedroite(longtopulse(113.2));
  delay(500);
  tournergauche(90);
  delay(500);
  lignedroite(longtopulse(71.2));
  delay(500); 
  tournerdroite(90);
  delay(500);
  delay(500);
  lignedroite(longtopulse(78.2));
  delay(500);
  tournerdroite(45);
  delay(500);
  lignedroite(longtopulse(166.2));
  delay(500);
  tournergauche(90);
  delay(500);
  lignedroite(longtopulse(71));
  delay(500);
  tournerdroite(45);
  delay(500);
  lignedroite(longtopulse(100));
  delay(10000);

  //Fonction pour le demi-tour

  //Fonction pour le retour
  lignedroite(longtopulse(109.3));
  delay(500);
  tournergauche(45);
  delay(500);
  lignedroite(longtopulse(57.8));
  delay(500);
  tournerdroite(90);
  delay(500);
  lignedroite(longtopulse(172.5));
  delay(500);
  tournergauche(45);
  delay(500);
  lignedroite(longtopulse(81.2));
  delay(500);
  tournergauche(90);
  delay(500);
  lignedroite(longtopulse(71.4));
  delay(500);
  tournerdroite(90);
  delay(500);
  lignedroite(longtopulse(113.2));
  
}
