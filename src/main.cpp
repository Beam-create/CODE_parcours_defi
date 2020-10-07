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

float kp = 0.002;//0.002;
float ki = 0.000001;

float vitesse=0.55;
long int distanceParcouruG;
long int distanceParcouruD;
 const int deltat=0.01*1000;
 long int nbCycle=0;
unsigned long temps=deltat;

/* ****************************************************************************
Vos propres fonctions sont creees ici
**************************************************************************** */
long int longtopulse(float distance)
{
  // code
  // entrer distance en cm
  //info: 3200 pulse / tours, 3" = 7.62cm, 23.94cm(7.62*pi)/tours
  long int nbpulse;
  nbpulse = ceil((3200/23.94)*distance);
  return nbpulse;
}


double pi(long int pulseAttendu, long int pulseReel,long int distanceParcouru)
{
  int erreur = pulseAttendu - pulseReel;
  
  long int erreurTotal = (pulseAttendu*nbCycle)-distanceParcouru;

  //double correction = (kp*erreur)+(ki*erreursum);

  

  return kp*erreur+ki*erreurTotal;
}


long int pulseParDeltaT(float vitesse,double deltat){
int rpmMax=200;
return ((vitesse*rpmMax*3200)/60)*deltat/1000;

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
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  //delay(10);// Delais pour décharger le CPU

float vitesseAjustementDroite;
 float vitesseAjustementGauche;
     long int pulseAttendu=pulseParDeltaT(vitesse,deltat);
      Serial.println("pulseAttendu=");
      Serial.println(pulseAttendu);
      //temps=millis()+deltat;
      //Serial.println(pi(ENCODER_Read(0),ENCODER_Read(1) ));

      if(millis()-temps>=deltat) {
        nbCycle++;
        distanceParcouruD= distanceParcouruD+ENCODER_Read(1);
        distanceParcouruG= distanceParcouruG+ENCODER_Read(0);
         vitesseAjustementGauche = vitesse + pi(pulseAttendu,ENCODER_Read(0),distanceParcouruG);
         vitesseAjustementDroite = vitesse + pi(pulseAttendu,ENCODER_Read(1),distanceParcouruD);
        
         ENCODER_Reset(0);
         ENCODER_Reset(1);

       Serial.println("vitesse droite=");
       Serial.println(vitesseAjustementDroite);
        Serial.println("vitesse gauche=");
       Serial.println(vitesseAjustementGauche);

        MOTOR_SetSpeed(1,vitesseAjustementDroite);
         MOTOR_SetSpeed(0,vitesseAjustementGauche);
         temps=millis();
      }
  
      }
  
   

    

   

  



