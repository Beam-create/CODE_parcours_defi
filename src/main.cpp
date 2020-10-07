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

//DISTANCE DU PARCOUR:
float distance1 = 122.5;

float kp = 0.002;
float ki = 0.000;
//float kd = 0.00;


float targetspeed =0.5;

float masterpps =0;
float slavepps = 0;
float correctedpps = 5300;
float correctedspeed =0.5;
unsigned long int temps = millis();

long int encodm1 =0;
long int encodm2 =0;
long int encods1 =0;
long int encods2 =0;

double erreursum =0;
const float deltat= 0.03;



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

/*
double pi(long int target, long int lecture)
{
  int erreur = target - lecture;
  
  erreursum = erreursum + (erreur*deltat);

  double correction = (kp*erreur)+(ki*erreursum);

return correction;
}
*/
double pi(float master, float slave)
{
  float erreur = master - slave;
  
  erreursum = erreursum + (erreur*deltat);

  double correction = (kp*erreur)+(ki*erreursum);
/*
  Serial.print("erreur :");
  Serial.print(erreur);
  Serial.print("\n");
  Serial.print("erreursum :");
  Serial.print(erreursum);
  Serial.print("\n");
*/
  return correction;
}
  

  


/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */


void setup(){
  BoardInit();
  Serial.begin(9600);

  ENCODER_Reset(0);
  ENCODER_Reset(1);
  
  
}



/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"
 
void loop() {
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  //delay(10);// Delais pour décharger le CPU
/*
  MOTOR_SetSpeed(0, targetspeed);
  MOTOR_SetSpeed(1, correctedspeed);

    encodm2= ENCODER_Read(0);
    encods2= ENCODER_Read(1);


  
  if(millis()-temps >= (deltat*1000))
  {
    
    Serial.print(encodm1);

    masterpps = (encodm2-encodm1)/deltat;
    slavepps = (encods2-encods1)/deltat;

    correctedpps = slavepps + pi(masterpps, slavepps);
    correctedspeed = correctedpps*(60/(deltat*3200*200));
    temps = millis();
    encodm1 = encodm2;
    encods1 = encods2;




    Serial.print("\nmaster :");
    Serial.print(ENCODER_ReadReset(0));
 
     Serial.print("\ndiférence master : ");
    Serial.print(encodm2-encodm1);
Serial.print("\ntemp :");
    Serial.print(temps/1000);
    
   
    Serial.print("\n");
    Serial.print("slave :");
    Serial.print(slavepps);
    Serial.print("\n");
    Serial.print("correction pps :");
    Serial.print(correctedpps);
    Serial.print("\n");
    Serial.print("correction final :");
    Serial.print(correctedspeed);
    Serial.print("\n");
  }
*/





int32_t encod0 =0;
int32_t encod1 =0; 

//long int distpulse1 = longtopulse(distance1); // Pour faire la distance de la premièere ligne droite, les roues font ce nbre de pulse
  
float mot0speed;
//float mot1speed;
    
int targetpulse = 160;
 
MOTOR_SetSpeed(0,targetspeed);
 
if(millis()-temps >= (deltat*1000))
  { 
    encod0 =ENCODER_Read(0);
    //encod1 =ENCODER_Read(1);
    double correction= pi(targetpulse, encod0);
    targetspeed = targetspeed + correction;
    //mot1speed = targetspeed + pi(targetpulse, encod1);
    //MOTOR_SetSpeed(0,mot0speed);
    //MOTOR_SetSpeed(1,mot1speed);
    temps = millis();

    Serial.print("\ncorrection :");
    Serial.print(correction);
      
  }
    

    /*
    MOTOR_SetSpeed(0,0);
    MOTOR_SetSpeed(0,0);
    ENCODER_Reset(0);
    ENCODER_Reset(1);
   

     long int erreurdistold;
    long int erreurdist;
    long int deltaerreurdist;

    long int erreurvit ;

    long int erreuraire ;

    erreurdistold=erreurdist;
    erreurdist= (encod0-encod1); //composante P 
    deltaerreurdist= (erreurdist-erreurdistold); // trouver le nbre de pulse  de différent entre les encodeurs

    erreurvit = (deltaerreurdist/deltat);// composante D vitesse en pulse/ms

    //erreuraire = (erreuraire + (erreurdist*deltat)); //composante I

    erreuraire = (erreuraire+ (erreurdist*deltat)); //erreur!!

    
    float mot1speed= targetspeed +(((kp*erreurdist)+(ki*erreuraire)+(kd*erreurvit)));
    float mot0speed= targetspeed ; //+(((kp*erreurdist)+(ki*erreuraire)+(kd*erreurvit))/2)
    
*/
   
    

   

  


}
