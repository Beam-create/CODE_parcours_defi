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

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978



/* **********************************************************************
Variables globales et defines
********************************************************************** */

//**********************************************************************

const double kp = 0.0015;// kp = 0.0015;
const double ki = 0.00008;//  ki = 0.00008;
const int deltat=75; //ms

const double distanceroues =18.5; // 18.26
long int erreurTotal=0;

unsigned int tempsdepause=120;
float vitesse=0.20;
long int distanceTotal = 0;

int alfred = 0;
int tour = 0;
int LedAllumer =0;
/* **********************************************************************
Vos propres fonctions sont creees ici
********************************************************************** */

long int distanceEnPulse(float distance)
{
  // entrer distance en cm
  return round((3200/23.94)*distance);
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
//*******************************************************************************
void suiveurtable()
{
MOTOR_SetSpeed(0, vitesse + pi(ENCODER_ReadReset(1), ENCODER_ReadReset(0))+(kp*PIDTable()));
MOTOR_SetSpeed(1, vitesse);
delay(deltat); 
return;
}

//***************************************************************************
void CompteurUstensils()
{
  if(LedAllumer==0)
  {
   digitalWrite(38,LOW);
   digitalWrite(40,LOW);
   digitalWrite(42,LOW);
   digitalWrite(44,LOW);
   digitalWrite(46,LOW);
   digitalWrite(48,LOW);
  }
  if(LedAllumer==1)
  {
   digitalWrite(38,HIGH);
   digitalWrite(40,LOW);
   digitalWrite(42,LOW);
   digitalWrite(44,LOW);
   digitalWrite(46,LOW);
   digitalWrite(48,LOW);
  }
  if(LedAllumer==2)
  {
  digitalWrite(38,HIGH);
   digitalWrite(40,HIGH);
   digitalWrite(42,LOW);
   digitalWrite(44,LOW);
   digitalWrite(46,LOW);
   digitalWrite(48,LOW);
  }
  if(LedAllumer==3)
  {
   digitalWrite(38,HIGH);
   digitalWrite(40,HIGH);
   digitalWrite(42,HIGH);
   digitalWrite(44,LOW);
   digitalWrite(46,LOW);
   digitalWrite(48,LOW);
  }
  if(LedAllumer==4)
  {
   digitalWrite(38,HIGH);
   digitalWrite(40,HIGH);
   digitalWrite(42,HIGH);
   digitalWrite(44,HIGH);
   digitalWrite(46,LOW);
   digitalWrite(48,LOW);
  }
  if(LedAllumer==5)
  {
   digitalWrite(38,HIGH);
   digitalWrite(40,HIGH);
   digitalWrite(42,HIGH);
   digitalWrite(44,HIGH);
   digitalWrite(46,HIGH);
   digitalWrite(48,LOW);
  }
  if(LedAllumer==6)
  {
   digitalWrite(38,HIGH);
   digitalWrite(40,HIGH);
   digitalWrite(42,HIGH);
   digitalWrite(44,HIGH);
   digitalWrite(46,HIGH);
   digitalWrite(48,HIGH);
  }
}

//*******************************************************************************
//***************************************************************************
int melody[] = {

  NOTE_D3, NOTE_D3, NOTE_CS3, NOTE_CS3, NOTE_C3, NOTE_C3, NOTE_CS3, NOTE_CS3, NOTE_G3, NOTE_G3
};
//*******************************************************************************
int noteDurations[] = {

  200, 200, 200, 200, 200, 200, 200 , 200, 400, 1600
};
//**********************************************************************************
void Buzzer() {

  for (int thisNote = 0; thisNote < 8; thisNote++) {
    tone(36, melody[thisNote], noteDurations[thisNote]);

    int pauseBetweenNotes = noteDurations[thisNote] * 1.35;

    delay(pauseBetweenNotes);

    noTone(36);

  }
}
//*****************************************************************************
void BuzzerFin() {

  for (int thisNote = 0; thisNote < 10; thisNote++) {
    tone(36, melody[thisNote], noteDurations[thisNote]);

    int pauseBetweenNotes = noteDurations[thisNote] * 1.35;

    delay(pauseBetweenNotes);

    noTone(36);

  }
}
//*********************************************************************************
void dropustensil()
{
  digitalWrite(43,HIGH);
  digitalWrite(39,LOW);
   SERVO_SetAngle(1,110);
  delay(1000);
  SERVO_SetAngle(1,0);
  delay(1000);
  LedAllumer= LedAllumer-1;
  CompteurUstensils();
  digitalWrite(43,LOW);
  digitalWrite(41,LOW);
  return;
}
//**********************************************************************************

void balayage()
{
  digitalWrite(39,LOW);
  digitalWrite(43,HIGH);
  SERVO_SetAngle(0,70);
  delay(1000);
  SERVO_SetAngle(0,180);
  digitalWrite(43,LOW);
  digitalWrite(39,HIGH);
  return;
}

//***************************************************************************
void AvancerApresChaise(float distancein)
{
  // Fonction pour avancer en ligne droite 
  // distancein : distance à parcourir (en cm)
  long int distanceParcouru=0;
  long int distance = distanceEnPulse(distancein);  
  ENCODER_Reset(0);
  ENCODER_Reset(1);
  
  while(distanceParcouru<distance)
  {
       //calcul distanceParcouru
      distanceParcouru= distanceParcouru+ENCODER_Read(1);
      digitalWrite(43,HIGH);

      //corrige la vitesse   
     suiveurtable();
    
    //mettre tourner bord de table
    if(ROBUS_ReadIR(0) < 150)
    {
      arreter();
      delay(1000);
      dropustensil();
      balayage();
      tournerGaucheSurLuiMeme(0.1, 90);
      tour= tour + 1;
        if(tour==4)
        {arreter();
          alfred=69;
          return;
        }
      alfred = 2;
      return;
    }


  }//fin while
    digitalWrite(43,LOW);
    arreter();
    return;
}

//***********************************************************************
//Fonctions d'initialisation (setup)
//***********************************************************************

void setup()
{
  pinMode(38, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(46, OUTPUT);
  pinMode(48, OUTPUT);

  pinMode(39, OUTPUT);
  pinMode(41, OUTPUT);
  pinMode(43, OUTPUT);
  
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT); 
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(A12, INPUT);
  pinMode(A13, INPUT);
  
  SERVO_SetAngle(0,180);
  SERVO_SetAngle(1,0);
  
  
  BoardInit();
  Serial.begin(9600);
}


/******************************************************************************
Fonctions de boucle infini (loop())
******************************************************************************/
void loop()
{

 if (digitalRead(26)==1)
    {
      LedAllumer = LedAllumer + 1;
      CompteurUstensils();
      delay(400);
    }
    if (digitalRead(27)==1)
    {
      LedAllumer = LedAllumer - 1;
      CompteurUstensils();
      delay(400);
    }

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
suiveurtable();
digitalWrite(39,HIGH);

    if (ROBUS_ReadIR(1)  > 85)
    { 
      alfred = 4;
      break;
    }

    if(ROBUS_ReadIR(0) < 150)
    {
      arreter();
      delay(500);
      balayage();
      tournerGaucheSurLuiMeme(0.1, 90);
      //tour = tour + 1;
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
    delay(1000); 
    dropustensil();
    //Buzzer();
    
    alfred =2;
  break;
//****************************************************************
case 4:
    if (ROBUS_ReadIR(1)  > 85)
    {
    digitalWrite(41,HIGH);  
    suiveurtable(); 

    if(ROBUS_ReadIR(0) < 150)
    {
      arreter();
      delay(1000);
      dropustensil();
      balayage();
      tournerGaucheSurLuiMeme(0.1, 90);
      //tour = tour + 1;
          if(tour==4)
          {
             arreter();
             alfred = 69;
           break;
          }
      alfred = 2;
      break;
    }
    }
    else
    {
      digitalWrite(41,LOW);
    alfred = 3; 
    AvancerApresChaise(10);
    }
    break;
   
    
//****************************************************************
case 69:

digitalWrite(39,HIGH);
digitalWrite(41,HIGH);
digitalWrite(43,HIGH);

int i;
int k;
for(i=0;i<2;i++)
{
  for(k=0;k<6;k++)
  {
  digitalWrite(38+(2*k),HIGH);
  delay(200);
  digitalWrite(38+(k*2),LOW);
  }
}
//Buzzer();
//BuzzerFin();
delay(1000);
digitalWrite(39,LOW);
digitalWrite(41,LOW);
digitalWrite(43,LOW);

alfred=70;
break;
//*******************************************************
case 70:
break;
//*******************************************************
default:

break;
  }//swith case

}//loop