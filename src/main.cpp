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

  double correction = (kp*erreur)+(ki*erreursum);

return correction;

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
/*
  MOTOR_SetSpeed(0, targetspeed);
  MOTOR_SetSpeed(1, correctedspeed);

    encodm2= ENCODER_Read(0);
    encods2= ENCODER_Read(1);

<<<<<<< HEAD

  
  if(millis()-temps >= (deltat*1000))
  {
    
    Serial.print(encodm1);

    masterpps = (encodm2-encodm1)/deltat;
    slavepps = (encods2-encods1)/deltat;
=======
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
  
<<<<<<< HEAD
      }
  
=======
  MOTOR_SetSpeed(0,0.5);
>>>>>>> bf9d292bf7e65f57bd8b8ffde8e72c6b87a5405b

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
    
<<<<<<< HEAD
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
=======
    int targetpulse = 500;
    int targetqty = ceil(distpulse1/500);
   for(int i=0; i<targetqty; i++)
   {
     if(millis()-time<=deltat)
     {
        mot0speed = targetspeed + pi(targetpulse, ENCODER_Read(0));
        mot1speed = targetspeed + pi(targetpulse, ENCODER_Read(1));

        Serial.println(encod0);
        
        MOTOR_SetSpeed(0,mot0speed);
        MOTOR_SetSpeed(1,mot1speed);
        time=millis();
     }
   }
 
      
    }
    //MOTOR_SetSpeed(0,0);
   // MOTOR_SetSpeed(0,0);
>>>>>>> bf9d292bf7e65f57bd8b8ffde8e72c6b87a5405b
    ENCODER_Reset(0);
    ENCODER_Reset(1);
>>>>>>> f5ca8017e276acefef11def76c41e2aad9e96289
   

<<<<<<< HEAD
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
=======
    

>>>>>>> bf9d292bf7e65f57bd8b8ffde8e72c6b87a5405b
   

  

<<<<<<< HEAD
  
=======
>>>>>>> bf9d292bf7e65f57bd8b8ffde8e72c6b87a5405b


