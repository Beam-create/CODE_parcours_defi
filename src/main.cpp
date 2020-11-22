/*
Projet: Le nom du script
Equipe: P24
Auteurs: Les membres auteurs du script
Description: Breve description du script
Date: Derniere date de modification */


/* **********************************************************************
Inclure les librairies de functions que vous voulez utiliser
********************************************************************** */

#include "LibRobus.h" // Essentielle pour utiliser RobUS
#include <stdio.h>
#include <math.h>



/* **********************************************************************
Variables globales et defines
********************************************************************** */
const double kp = 0.0015;// kp = 0.0015;
const double ki = 0.00008;//  ki = 0.00008;
const int deltat=75; //ms
const int deltatPID=50; //ms

const double distanceroues =18.5; // 18.26

int etape=0; 
long int erreurTotal=0;

unsigned int tempsdepause=120;
float vitesse=0.35;
float vitesseTourner=0.25;
float vitesseDepart = 0.15;
long int distanceTotal = 0;


int temps1 =0;
int temps2 =0;
int temps3 =0;
int temps4 =0;
int temps5 =0;
int temps6 =0;
int temps7 =0;
//quille
float limiteQuille= 60;
float distanceDepart=30;
float distanceQuille;

int micpin = A5; //pin micro
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
int PIDLigne()
{
int erreur1 = 10;
int erreur2 = 12;
int erreur3 = 15;
int erreur4 = 18;
int erreur0 = 0;

int A = 300; 
int B = 300; 
int C = 300; 
int D = 300; 
int E = 300; 
int F = 300; 
int G = 300; 
int H = 300; 
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
        erreurSuiveur = -erreur4;  
      }

return erreurSuiveur;
}

/* **********************************************************************
Fonctions d'initialisation (setup)
********************************************************************** */

void setup(){
  BoardInit();
  Serial.begin(9600);
}

/* **********************************************************************
Fonctions de boucle infini (loop())
********************************************************************** */

void loop() {
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  delay(10);// Delais pour décharger le CPU


  if(digitalRead(28)==1)
    {
      etape =1;
    }
    /*
  if(analogRead(micpin)>=600)
    {
      etape =1;
    }
*/
  switch (etape)
  {


//********************************************DEBUT SWITCH
  case 1:
    
    avancerDistance(0.2, distanceDepart);
    distanceTotal+=distanceEnPulse(distanceDepart);
    arreter();
    etape=2;
    delay(1500);
    break;
//********************************************************
  case 2:
   
    if(SONAR_GetRange(0)<limiteQuille)
    { 
      tournerGaucheSurLuiMeme(0.2, 90);
      avancerDistance(vitesse, 65);
      tournerDroiteSurLuiMeme(0.2, 90);
      avancerDistance(vitesse, 40);
      tournerGaucheSurLuiMeme(0.2, 90);
      etape=69;
    }else{
      etape=3;
    }
    break;


//********************************************************
  case 3:

    avancer(vitesse);
    etape=4;
    ENCODER_Reset(0);
    ENCODER_Reset(1);
    break;

//*******************************************************
  case 4:

    if(pulseEnDistance(distanceTotal)<175)  
    {
      if(millis()-temps1>=deltatPID)
      {
        distanceTotal+=ENCODER_Read(1);
        MOTOR_SetSpeed(0,vitesse+pi(ENCODER_ReadReset(1), ENCODER_ReadReset(0))+(kp*PIDLigne()));//+(kp * PIDLigne())
        MOTOR_SetSpeed(1,vitesse);
        temps1=millis();
      }
        if(millis()-temps4>=deltat)
        {
         distanceQuille=SONAR_GetRange(0);
          if(distanceQuille<limiteQuille){
           etape=5;
           break;
          }
        temps4=millis();
        }
    }else{
      arreter(); 
      etape=49;
      break;
    }
    break;


//**************************************************************************
case 49 : 

if(pulseEnDistance(distanceTotal)<209)  //total 475
{
   if(millis()-temps2>=deltatPID)
   { 
      distanceTotal+=ENCODER_Read(1);
      MOTOR_SetSpeed(0,vitesse+pi(ENCODER_ReadReset(1), ENCODER_ReadReset(0))+(kp*PIDLigne()));
      MOTOR_SetSpeed(1,vitesse);
      temps2=millis();
    }
      if(millis()-temps6>=deltat)
      {
        distanceQuille=SONAR_GetRange(0);
          if(distanceQuille<35)
          {
            etape=5;
            break;
          }
      temps6=millis();
      }
}else{
arreter();
etape=50;
break;
}

break;


//*********************************************************************
  case 5:
    
    avancerDistance(0.5,distanceQuille*sin(55/2*PI/180));
    distanceTotal+=distanceEnPulse(distanceQuille*sin(55/2*PI/180));
    delay(tempsdepause);
    tournerGaucheSurLuiMeme(0.25,88);

    if(distanceTotal<distanceEnPulse(350))
    {
        avancerDistance(0.5,75);
        etape=69;
    }else{
        avancerDistance(0.5,65);
        delay(tempsdepause);
        tournerDroiteSurLuiMeme(0.25,90);
        etape=66;
        break;
    }

//************************************************************************
  case 6 :
        if(pulseEnDistance(distanceTotal)<472)  
        {
            if(millis()-temps3>=deltat)
            {
            distanceTotal+=ENCODER_Read(1);
            MOTOR_SetSpeed(0,vitesse+pi(ENCODER_ReadReset(1), ENCODER_ReadReset(0))+(kp*PIDLigne()));//+(kp * PIDLigne())
            MOTOR_SetSpeed(1,vitesse);
            temps3=millis();
            }
              if(millis()-temps7>=deltat)
              {
                   distanceQuille=SONAR_GetRange(0);
                   if(distanceQuille<limiteQuille)
                   {
                       Serial.println(distanceQuille);
                       etape=5;
                       break;
                   }
                temps7=millis();
              }
        }else{
        arreter();
        delay(tempsdepause);
        etape=100;
        break;
        }
        break;


//***********************************************
  case 50:
    delay(14000);
    etape=6;
    break;

//************************************************
  case 66:
    Serial.println(pulseEnDistance(distanceTotal));
    avancerDistance(vitesse,475-pulseEnDistance(distanceTotal));
    etape=69;
    break;

//************************************************
  case 69:
  //loop infini
    break;

//************************************************
  case 100:

    tournerGaucheSurLuiMeme(0.2,90);
    avancerDistance(vitesse,65);
    etape=69;
    break;
//************************************************
  default:
    
    break;
  }//fin switch case
  
}//fin loop
 
