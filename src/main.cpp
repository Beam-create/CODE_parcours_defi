/*
Projet: Le nom du script
Equipe: P24
Auteurs: equipe 24
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

const double kp = 0.0015;// kp = 0.0015;
const double ki = 0.00008;//  ki = 0.00007;
const int deltat=75; //ms
const double distanceroues =18.7; // 18.26

int state=1; 
long int erreurTotal=0;

int tempsdepause=200;
float vitesse=0.5;
float vitesseTourner=0.3;


/* ****************************************************************************
Vos propres fonctions sont creees ici
**************************************************************************** */

long int distanceEnPulse(float distance)
{
  // entrer distance en cm
  //info: 3200 pulse / tours, 3" = 7.62cm, 23.94cm(7.62*pi)/tours
  long int nbpulse;
  nbpulse = round((3200/23.94)*distance);
  return nbpulse;
}
//**********************************************************************************************
double pi(long int pulseAttendu, long int pulseReel)
{
  int erreur = pulseAttendu - pulseReel;
  erreurTotal += erreur;
  return kp*erreur+ki*erreurTotal;
}
//************************************************************************************************
double AngleEnPulse (int angle) {
Serial.println(PI*2*distanceEnPulse(distanceroues)*angle/360.0);
return PI*2*distanceEnPulse(distanceroues)*angle/360.0;

}
//*****************************************************************************************************
void avancer(float vitesse, long int distance){
long int distanceParcouru=0;
float vitesseDepart=vitesse;
float a=0,d=0;
long int distancePrecedente=0;
float nbIteration=10.0;

//MOTOR_SetSpeed(1,vitesse);
//MOTOR_SetSpeed(0,vitesse);

 while(distanceParcouru<distance){
  
 //a chaque deltaT
 delay(deltat);

 //calcule distanceParcouru
 distanceParcouru= distanceParcouru+ENCODER_Read(1);

 //acceleration
 if(distanceParcouru<(distance/2.0)){
    if(distanceParcouru<=distancePrecedente+(distance/2.0)/nbIteration){
        distancePrecedente=distanceParcouru;
        vitesse=-1*(vitesseDepart)*pow(0.20,a)+vitesseDepart; // fonction exponentiel y=ac^x+k
        a=a+(1/nbIteration);
        
    }
  }
 //decceleration
 if(distanceParcouru>(distance/2.0)){
    if(distanceParcouru>=distancePrecedente+(distance/2.0)/nbIteration){
        distancePrecedente=distanceParcouru;
        vitesse=vitesseDepart*pow(0.20,d);
        d=d+(1/nbIteration);   
    }
  }

 //corrige la vitesse   
 MOTOR_SetSpeed(0,vitesse + pi(ENCODER_ReadReset(1),ENCODER_ReadReset(0)));  
 MOTOR_SetSpeed(1,vitesse);
 }//fin while

MOTOR_SetSpeed(0,0);
MOTOR_SetSpeed(1,0);
delay(tempsdepause);

}

//************************************************************************************************************

void tournerDroite(float vitesse, int angle){
long int distanceParcouru=0;

MOTOR_SetSpeed(1,0);
MOTOR_SetSpeed(0,vitesse);

while(distanceParcouru<AngleEnPulse(angle)){
distanceParcouru= distanceParcouru+ENCODER_ReadReset(0);   
}
MOTOR_SetSpeed(0,0);
delay(tempsdepause);

}
  
//********************************************************************************************************************

void tournerGauche(float vitesse,int angle){
long int distanceParcouru=0;

MOTOR_SetSpeed(0,0);
MOTOR_SetSpeed(1,vitesse);

while(distanceParcouru<AngleEnPulse(angle)){
distanceParcouru= distanceParcouru+ENCODER_ReadReset(1);   
}

MOTOR_SetSpeed(1,0);
delay(tempsdepause);

}
//********************************************************************************************************************

int angleAPulse(int angle)
{
float circonference= distanceroues * M_PI;
float distanceafaire= circonference * angle / 360.0;
float angleEnPulse = distanceEnPulse(distanceafaire);
return angleEnPulse;
}
//****************************************************************************************************************

void tournerDroiteSurLuiMeme(float vitesseT ,int angle){
  long int distanceParcourue = 0;
  float angleEnPulse= angleAPulse(angle);

  MOTOR_SetSpeed(1,-(vitesseT));
  MOTOR_SetSpeed(0,vitesseT);

  while( distanceParcourue<angleEnPulse ){
    delay(deltat);
    distanceParcourue= distanceParcourue+ENCODER_Read(0);
    MOTOR_SetSpeed(1, -(vitesseT + pi(ENCODER_ReadReset(0),-(ENCODER_ReadReset(1)))));
  }
  MOTOR_SetSpeed(1,0);
  MOTOR_SetSpeed(0,0);
  delay(tempsdepause);
}
//**************************************************************************************************************

void tournerGaucheSurLuiMeme(float vitesseT, int angle){
  long int distanceParcourue = 0;
  float angleEnPulse= AngleEnPulse(angle);

  MOTOR_SetSpeed(1,vitesseT);
  MOTOR_SetSpeed(0,-(vitesseT));

  while( distanceParcourue  < angleEnPulse ){
    delay(deltat);
    distanceParcourue= distanceParcourue+ -(ENCODER_Read(0));
    MOTOR_SetSpeed(0,vitesseT + pi(-(ENCODER_ReadReset(0)),ENCODER_ReadReset(1)));
  }
  MOTOR_SetSpeed(1,0);
  MOTOR_SetSpeed(0,0);
  delay(tempsdepause);
}
/* *********************************************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */


void setup(){
  BoardInit();
  Serial.begin(9600);
}

/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

void loop() {

if(state){

  avancer(vitesse,distanceEnPulse(112.5));
  
  tournerGauche(vitesseTourner,90);
  
  avancer(vitesse,distanceEnPulse(71.2));
   
  tournerDroite(vitesseTourner,90);
  
  avancer(vitesse,distanceEnPulse(73)); //78.2
  
  tournerDroite(vitesseTourner,44); //44
  
  avancer(vitesse,distanceEnPulse(170)); //166.2
  
  tournerGauche(vitesseTourner,90);
  
  avancer(vitesse,distanceEnPulse(55));
  
  tournerDroite(vitesseTourner,45);
  
  avancer(vitesse,distanceEnPulse(111));

  tournerDroiteSurLuiMeme(0.1, 180);

  avancer(vitesse,distanceEnPulse(111));

  tournerGauche(vitesseTourner,45);

  avancer(vitesse,distanceEnPulse(50));

  tournerDroite(vitesseTourner,92);

  avancer(vitesse,distanceEnPulse(170));

  tournerGauche(vitesseTourner,43);

  avancer(vitesse,distanceEnPulse(73));

  tournerGauche(vitesseTourner,90);

  avancer(vitesse,distanceEnPulse(71.2));

  tournerDroite(vitesseTourner,90);

  avancer(vitesse,distanceEnPulse(127.5));//112.5

  state=0;
  


}

  
}

        
         
     




      
   

    

   

  




