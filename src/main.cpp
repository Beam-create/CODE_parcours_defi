/*
Projet: Le nom du script
Equipe: P24
Auteurs: Les membres auteurs du script
Description: Breve description du script
Date: Derniere date de modification
*/

/* **********************************************************************
Inclure les librairies de functions que vous voulez utiliser
********************************************************************** */

#include "LibRobus.h" // Essentielle pour utiliser RobUS
#include <stdio.h>
#include <math.h>



/* **********************************************************************
Variables globales et defines
********************************************************************** */

//**********************************************************************

const double kp = 0.0015;// kp = 0.0015;
const double ki = 0.00008;//  ki = 0.00008;
const int deltat=75; //ms
const int deltatPID=50; //ms

const double distanceroues =18.5; // 18.26
const double largeurAlfred = 14.5;

int etape=0; 
long int erreurTotal=0;

//**********
const double distanceTable=200;
const double distanceSiege = 0;
double distanceParcourue = 0;
double temps = 0;
//**********************************************************************

unsigned int tempsdepause=120;
float vitesse=0.23;
float vitesseTourner=0.25;
float vitesseDepart = 0.15;
long int distanceTotal = 0;

//*********************************************************************

int temps1 =0;
int temps2 =0;
int temps3 =0;
int temps4 =0;
int temps5 =0;
int temps6 =0;
int temps7 =0;

//********************************************************************

float limiteQuille= 60;
float distanceDepart=30;
float distanceQuille;

int micpin = A5; //pin micro

//*******************************************************************

int alfred = 0;
int tour = 0;
/* **********************************************************************
Vos propres fonctions sont creees ici
********************************************************************** */

long int distanceEnPulse(float distance)
{
  // entrer distance en cm
  return round((3200/23.94)*distance);
}
//**********************************************************************************************
int pulseEnDistance(float pulse)
{
  return (23.94/3200)*pulse;
}
//**********************************************************************************************
double pi(long int pulseAttendu, long int pulseReel)
{
  int erreur = pulseAttendu - pulseReel;
  erreurTotal += erreur;
  return kp*erreur+ki*erreurTotal;
}
//************************************************************************************************
double AngleEnPulse (int angle) 
{
  return PI*2*distanceEnPulse(distanceroues)*angle/360.0;
}
//************************************************************************************************
void avancerDistance(float vitessein, float distancein)
{
  // Fonction pour avancer en ligne droite avec accéleration et deccéleration.
  // vitessein : vitesse max (entre 0 et 1)
  // distancein : distance à parcourir (en cm)
  long int distanceParcouru=0;
  long int distance = distanceEnPulse(distancein);
  unsigned long int temps =0;
  float vitesselive = 0.35;
  float accel=floor((6.0/475.0)*distancein-(5.0/19.0)); 
  ENCODER_Reset(0);
  ENCODER_Reset(1);
  
  while(distanceParcouru<distance)
  {
 
    if(millis()-temps >= deltat)
    {

      //calcul distanceParcouru
      distanceParcouru= distanceParcouru+ENCODER_Read(1);

      if(distancein>=100)
      {
        //acceleration
        if(distanceParcouru<=(distance/(accel*4.0)))
        {
          vitesselive=(((-1*vitessein)/2)+(vitesseDepart/2))*cos( accel * M_PI * (distanceParcouru*4.0/distance) )+((vitessein/2)+(vitesseDepart/2));  
        }
        //decceleration
        if(distanceParcouru>=(((accel*4.0)-1)*distance/(accel*4.0)))
        {
          vitesselive=(((-1*vitessein)/2)+(vitesseDepart/2))*cos( accel * M_PI * (distanceParcouru*4.0/distance) )+((vitessein/2)+(vitesseDepart/2));
        }
      }
      //corrige la vitesse   
      MOTOR_SetSpeed(0,vitesselive + pi(ENCODER_ReadReset(1),ENCODER_ReadReset(0)));  
      MOTOR_SetSpeed(1,vitesselive);
      temps = millis();
    }
  }//fin while
  do
  {
    MOTOR_SetSpeed(0,0);
    MOTOR_SetSpeed(1,0);
  } while (millis()-temps<tempsdepause);


}

//************************************************************************************************ 
void arreter()
{
  MOTOR_SetSpeed(0,0);
  MOTOR_SetSpeed(1,0);
}
//************************************************************************************************ 
void avancer(float vitessein)
{
  MOTOR_SetSpeed(0,vitessein);
  MOTOR_SetSpeed(1,vitessein);
}
//************************************************************************************************ 
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
//************************************************************************************************ 

void tournerDroiteSurLuiMeme(float vitesseT ,int angle){
  long int distanceParcourue = 0;
  float angleEnPulse= AngleEnPulse(angle)/2;
  ENCODER_Reset(0);
  ENCODER_Reset(1);

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
  float angleEnPulse= AngleEnPulse(angle)/2;
  ENCODER_Reset(0);
  ENCODER_Reset(1);

  MOTOR_SetSpeed(1,vitesseT);
  MOTOR_SetSpeed(0,-(vitesseT));

  while( distanceParcourue  < angleEnPulse ){
    delay(deltat);
    distanceParcourue= distanceParcourue+ ENCODER_Read(1);
    MOTOR_SetSpeed(0,-(vitesseT + pi(ENCODER_ReadReset(1),-(ENCODER_ReadReset(0)))));
  }
  MOTOR_SetSpeed(1,0);
  MOTOR_SetSpeed(0,0);
  delay(tempsdepause);
}
//**************************************************************************************************************
float PIDTable()
{
int erreur1 = 10;
int erreur2 = 15;
int erreur3 = 17;
int erreur4 = 20;
int erreur0 = 0;

int A = 780; 
int B = 780; 
int C = 780; 
int D = 780; 
int E = 780; 
int F = 780; 
int G = 780; 
int H = 780; 
int erreurSuiveur ;


      if(analogRead(6) < A) 
      {
        erreurSuiveur = erreur4;
      }  
      if(analogRead(6) >= A && analogRead(7) < B) 
      {
        erreurSuiveur = erreur3;  
      }
      if(analogRead(7) >= B && analogRead(8) < C) 
      {
        erreurSuiveur = erreur2;  
      }
      if(analogRead(8) >= C && analogRead(9) < D) 
      {
        erreurSuiveur = erreur1;  
      }
      if(analogRead(9) >= D && analogRead(10) < E) 
      {
        erreurSuiveur = erreur0;  
      }
      if(analogRead(10) >= E && analogRead(11) < F) 
      {
        erreurSuiveur = -erreur1;  
      }
      if(analogRead(11) >= F && analogRead(12) < G) 
      {
        erreurSuiveur = -erreur2;  
      }
      if(analogRead(12) >= G && analogRead(13) < H) 
      {
        erreurSuiveur = -erreur3;  
      }
      if(analogRead(13) > H) 
      {
        erreurSuiveur = erreur4;  
      }
      if(analogRead(6) >= A && analogRead(7) > B && analogRead(8) > C && analogRead(9) > D && analogRead(10) > E && analogRead(11) > F && analogRead(12) > G  && analogRead(13) > H ) 
      {
        erreurSuiveur = -erreur4;
      }

  return erreurSuiveur;  
 
}
/************************************************************************
 * Fonction de détection des sièges
 * *********************************************************************/
int siegeOuNon (int oui, int non)
{
  int valeur = 0;
  if (millis()-temps2 >=100)
  {
    if (SONAR_GetRange(0)<=67.4)
    {
      valeur = oui;
    }
    else if (SONAR_GetRange(0)> 67.4)
    {
      valeur = non;
    }
  temps2 = millis();
  }
  return valeur;
}
//******************************************************************************
int deplacementBord()
{
  int valeur = 0;
  ENCODER_Reset(0);
  ENCODER_Reset(1);
  if (millis()- temps1 > deltatPID)
  {
    MOTOR_SetSpeed(1,vitesse);
    MOTOR_SetSpeed(0, vitesse + pi(ENCODER_ReadReset(1),ENCODER_ReadReset(0))+(kp*PIDTable()));
    temps1 = millis();
  }
  valeur =siegeOuNon(2,1);
  return valeur;
}
//************************************************************************************
int deplacementSiege()
{
  
  int valeur = 0;
  ENCODER_Reset(0);
  ENCODER_Reset(1);

  if (millis()- temps1 > deltatPID)
  {
    MOTOR_SetSpeed(1,vitesse);
    MOTOR_SetSpeed(0, vitesse + pi(ENCODER_ReadReset(1),ENCODER_ReadReset(0))+(kp*PIDTable()));
    temps1 = millis();
  }
  valeur =siegeOuNon(2,3);
  return valeur;
}
//***************************************************************************
//***********************************************************************
//Fonctions d'initialisation (setup)
//***********************************************************************

void setup()
{
  pinMode(39, OUTPUT);
  pinMode(41, OUTPUT);
  pinMode(43, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT); 
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(A12, INPUT);
  pinMode(A13, INPUT);
  BoardInit();
  Serial.begin(9600);
}


/******************************************************************************
Fonctions de boucle infini (loop())
******************************************************************************/
void loop()
{
  if (digitalRead(28)==1)
  {
    alfred = 1;
  }

  switch (alfred)
  {
 //****************************************************************   
case 1:
avancer(vitesse);
alfred=2;
break;
//***************************************************************
  case 2:
MOTOR_SetSpeed(0, vitesse + pi(ENCODER_ReadReset(1), ENCODER_ReadReset(0))+(kp*PIDTable()));
MOTOR_SetSpeed(1, vitesse);
delay(deltat); 

    if(ROBUS_ReadIR(0) < 150)
    {
      MOTOR_SetSpeed(0,0);
      MOTOR_SetSpeed(1,0);
      delay(500);
      tournerGaucheSurLuiMeme(0.1, 90);
      tour= tour + 1;
      avancer(vitesse);
    }

    if (SONAR_GetRange(0)<=67.4)
    {
     alfred = 3;
     break;
    }

    if(tour==4)
    {
      arreter();
      alfred = 69;
    }
break;
//********************************************************************
case 3:
arreter();
delay(deltat); 
//fonction depot dustensils
alfred =2;
break;
//****************************************************************
case 69:
break;
//*******************************************************
default:

break;
  }//swith case
}//loop