/*
Projet: Le nom du script
Equipe: P24
Auteurs: Tony
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

int etape=1;
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
double distancetourner (int angle) {
  Serial.println(PI*2*distanceEnPulse(distanceroues)*angle/360.0);
return PI*2*distanceEnPulse(distanceroues)*angle/360.0;

}
//*****************************************************************************************************
void avancer(float vitesse, long int distance){
long int distanceParcouru=0;
float vitesseDepart=vitesse;
float a=0,d=0;
long int distancePrecedente=0;

//MOTOR_SetSpeed(1,vitesse);
//MOTOR_SetSpeed(0,vitesse);

while(distanceParcouru<distance){
  
      //a chaque deltaT
      delay(deltat);

      //calcule distanceParcouru
      distanceParcouru= distanceParcouru+ENCODER_Read(1);

      //acceleration
      if(distanceParcouru<(distance/2.0)){
        if(distanceParcouru<=distancePrecedente+(distance/2.0)/10.0){
          distancePrecedente=distanceParcouru;
          vitesse=-1*(vitesseDepart)*pow(0.20,a)+vitesseDepart;
          a=a+0.1;
        
        }
      }
      //decceleration
      if(distanceParcouru>(distance/2.0)){
        if(distanceParcouru>=distancePrecedente+(distance/2.0)/10.0){
          distancePrecedente=distanceParcouru;
          vitesse=vitesseDepart*pow(0.20,d);
          d=d+0.1;
        
        }
      }
     
      //corrige la vitesse   
      MOTOR_SetSpeed(0,vitesse + pi(ENCODER_ReadReset(1),ENCODER_ReadReset(0)));  
      MOTOR_SetSpeed(1,vitesse);
}//fin while

MOTOR_SetSpeed(0,0);
MOTOR_SetSpeed(1,0);
delay(tempsdepause);

}// fin fonction

//************************************************************************
void tournerDroite(float vitesse, int angle){
long int distanceParcouru=0;

MOTOR_SetSpeed(1,0);
MOTOR_SetSpeed(0,vitesse);

while(distanceParcouru<distancetourner(angle)){
distanceParcouru= distanceParcouru+ENCODER_ReadReset(0);   

}//fin while

MOTOR_SetSpeed(0,0);
delay(tempsdepause);

}// fin fonction
  

void tournerGauche(float vitesse,int angle){
long int distanceParcouru=0;

MOTOR_SetSpeed(0,0);
MOTOR_SetSpeed(1,vitesse);

while(distanceParcouru<distancetourner(angle)){
distanceParcouru= distanceParcouru+ENCODER_ReadReset(1);   
}//fin while

MOTOR_SetSpeed(1,0);
delay(tempsdepause);

}// fin fonction
/* ****************************************************************************
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

if(etape==1){

  avancer(vitesse,distanceEnPulse(500));
  
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

  tournerDroite(0.6,1080);
  
  etape++;
  


}

  
}

        
         
     




      
   

    

   

  




