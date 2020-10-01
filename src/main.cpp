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



/* ****************************************************************************
Vos propres fonctions sont creees ici
**************************************************************************** */
long int longtopulse(float distance)
{
  // code
  // entrer distance en cm
  //info: 3200 pulse / tours, 3" = 7.62cm, 23.94cm(7.62*pi)/tours
  double pulse;
  long int nbpulse;
  pulse =(3200/23.94)*distance;
  nbpulse = ceil(pulse);
  return nbpulse; 
}



/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */

// VARIABLES GLOBALES:

//DISTANCE DU PARCOUR:
float distance1 = 122.5; // distance de la première ligne droite

int32_t encod0 =0;
int32_t encod1 =0;
float targetspeed =0.25;


//int t1= millis(); // dt en ms ?

//**********************************************

void setup(){
  BoardInit();
  Serial.begin(9600);

  
}


/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

void loop() {
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  delay(10);// Delais pour décharger le CPU
  
  
  
  
long int distpulse1 = longtopulse(distance1); // Pour faire la distance de la premièere ligne droite, les roues font ce nbre de pulse
  
  

  encod0= ENCODER_Read(0);
  encod1= ENCODER_Read(1);
 // MOTOR_SetSpeed(0,0);
  //MOTOR_SetSpeed(1,0);

  while(encod0<distpulse1 || encod1<distpulse1)
  {
    float kp = 0.0002;
    float ki = 0.000;// ????

    
    //int t2= t1;
    //t1=millis();
    //int deltat= t1-t2;
    int deltat=20;
    deltat = millis();


    long int erreurdistold; // ancienne erreur de pulse précédent
    long int erreurdist; // erreur en pulse
    long int deltaerreurdist;// différence de ce qui est lu par les deux encodeurs etla distance de l'intégrale

    long int erreurvit ;

    long int erreuraire ;

    erreurdistold=erreurdist;
    erreurdist= (encod0-encod1); //composante P 
    deltaerreurdist= (erreurdist-erreurdistold); // trouver le nbre de pulse  de différent entre les encodeurs

    erreurvit = (deltaerreurdist/deltat);// composante D vitesse en pulse/ms

    //erreuraire = (erreuraire + (erreurdist*deltat)); //composante I

    erreuraire = (erreuraire+ (erreurdist*deltat)); //erreur!!

    
    float mot1speed= targetspeed +(((kp*erreurdist)+(ki*erreuraire)));
    float mot0speed= targetspeed +(((kp*erreurdist)+(ki*erreuraire))); //bonne modification?


   
    MOTOR_SetSpeed(LEFT, mot0speed);
    MOTOR_SetSpeed(RIGHT, mot1speed);
    encod0 = ENCODER_Read(0);
    encod1 = ENCODER_Read(1);

    delay(20);

   

  }
Serial.print("la distance voulue est : ");
//Serial.print(distpulse1);
Serial.print("la distance actuelle est : ");
//Serial.print( encod0);
}
