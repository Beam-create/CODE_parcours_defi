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
float vitesseAjustementGauche=vitesse;
long int distanceParcouruG =0;
long int distanceParcouruD =0;
const int deltat=50;
long int nbCycle=0;
unsigned long temps=deltat;

long int erreurTotal=0;

long int turnparcouru =0;


/* ****************************************************************************
Vos propres fonctions sont creees ici
**************************************************************************** */
long int longtopulse(float distance)
{
  // entrer distance en cm
  //info: 3200 pulse / tours, 3" = 7.62cm, 23.94cm(7.62*pi)/tours
  long int nbpulse;
  nbpulse = ceil((3200/23.94)*distance);
  return nbpulse;
}

long int angletopulse(float anglein)
{
  //La fonction assume que le robot a une roue fixe.
  float rayon = 18.7;// distance c/c entre les roues (cm): 
  float anglerad = anglein *(M_PI/180);
  float arclong = anglerad*rayon;
  long int nbpulsearc = longtopulse(arclong);
  return nbpulsearc;
}


double pi(long int pulseAttendu, long int pulseReel,long int distanceParcouru)
{
  int erreur = pulseAttendu - pulseReel;
  
  //long int erreurTotal = (pulseAttendu*nbCycle)-distanceParcouru;

  erreurTotal = erreurTotal + erreur*(deltat/1000);

  double correction = (kp*erreur)+(ki*erreurTotal);

return correction;

}

long int pulseParDeltaT(float vitesse,double deltat)
{
int rpmMax=245;
return ((vitesse*rpmMax*3200)/60)*deltat/1000;
}

void lignedroite(long int distance) //, int step
{
  while(distanceParcouruG<distance ) //&& step==0
  {
    MOTOR_SetSpeed(1,0);
    MOTOR_SetSpeed(1,constrain(vitesseAjustementDroite,-1,1));
    MOTOR_SetSpeed(0,0);
    MOTOR_SetSpeed(0,vitesse);

    if(millis()-temps>=deltat) 
    {
      nbCycle++;
      distanceParcouruD= distanceParcouruD+ENCODER_Read(1);
      distanceParcouruG= distanceParcouruG+ENCODER_Read(0);
    
      //vitesseAjustementDroite = vitesse + pi(ENCODER_Read(0),ENCODER_Read(1),distanceParcouruD);
      vitesseAjustementDroite = vitesseAjustementDroite + pi(ENCODER_Read(0),ENCODER_Read(1),distanceParcouruD);
        
      /*     
      Serial.println("vitesse theorique=");
      Serial.println(vitesse);
      Serial.println("vitesse droite=");
      Serial.println(vitesseAjustementDroite);
      Serial.println("vitesse gauche=");
      Serial.println(vitesseAjustementGauche);
      Serial.println("pulseAttendu=");
      //Serial.println(pulseAttendu);
      Serial.println("vitesse theorique=");
      Serial.println(ENCODER_ReadReset(0));
      */
      ENCODER_Reset(0);
      ENCODER_Reset(1);
  
      temps=millis();
    }
  }
 if(1)//step==0
 {
   MOTOR_SetSpeed(1,0);
   MOTOR_SetSpeed(0,0);
   distanceParcouruD =0;
   distanceParcouruG =0;

   //step++;
 }
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



  //long int pulseAttendu=pulseParDeltaT(vitesse,deltat);

  long int distance1 = longtopulse(122.5);
  long int turn1 = angletopulse(90);

  //temps=millis()+deltat;
  //Serial.println(pi(ENCODER_Read(0),ENCODER_Read(1) ));

  lignedroite(longtopulse(122.5));
  delay(2000);
  lignedroite(longtopulse(160));
  
/*
  Serial.println("distance1=");
  Serial.println(distance1);
  Serial.println("distance parcourue");
  Serial.println(distanceParcouruG);

  while(distanceParcouruG<distance1 && etape==0)
  {
    MOTOR_SetSpeed(1,0);
    MOTOR_SetSpeed(1,constrain(vitesseAjustementDroite,-1,1));
    MOTOR_SetSpeed(0,0);
    MOTOR_SetSpeed(0,vitesse);

    if(millis()-temps>=deltat) 
    {
      nbCycle++;
      distanceParcouruD= distanceParcouruD+ENCODER_Read(1);
      distanceParcouruG= distanceParcouruG+ENCODER_Read(0);
    
      //vitesseAjustementDroite = vitesse + pi(ENCODER_Read(0),ENCODER_Read(1),distanceParcouruD);
      vitesseAjustementDroite = vitesseAjustementDroite + pi(ENCODER_Read(0),ENCODER_Read(1),distanceParcouruD);
        
          
      Serial.println("vitesse theorique=");
      Serial.println(vitesse);
      Serial.println("vitesse droite=");
      Serial.println(vitesseAjustementDroite);
      Serial.println("vitesse gauche=");
      Serial.println(vitesseAjustementGauche);
      Serial.println("pulseAttendu=");
      //Serial.println(pulseAttendu);
      Serial.println("vitesse theorique=");
      Serial.println(ENCODER_ReadReset(0));
      
      ENCODER_Reset(0);
      ENCODER_Reset(1);
  
      temps=millis();
    }
  }
 if(etape==0)
 {
   MOTOR_SetSpeed(1,0);
   MOTOR_SetSpeed(0,0);
   etape++;
 }
  */
  while(etape==1 && turnparcouru < turn1)
  {

  }
}
