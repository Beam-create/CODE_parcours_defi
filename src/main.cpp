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
#include <Wire.h>
#include "Adafruit_TCS34725.h"
//#define redpin 3
//#define greenpin 5
//#define bluepin 6
#define commonAnode true
byte gammatable[256];



/* ****************************************************************************
Variables globales et defines
**************************************************************************** */
// -> defines...
// L'ensemble des fonctions y ont acces

const double kp = 0.0015;// kp = 0.0015;
const double ki = 0.00008;//  ki = 0.00008;
const int deltat=75; //ms
const double distanceroues =18.7; // 18.26

int state=1; 
long int erreurTotal=0;

int tempsdepause=200;
float vitesse=0.5;
float vitesseTourner=0.3;
volatile float r, y, b;

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
// *********************************************************************************************************
void suiveurdeligne(float vitesse, long int distance)
{
  while(ENCODER_Read(0)<distance)
  {
    //avancer(vitesse,distance);
 
   if(analogRead(A10) < 100)
    {
      Serial.println(analogRead(A10));
      tournerDroite(0.2,10);
    }
    else if(analogRead(A10) >= 100)
    {
      Serial.println(analogRead(10));
      tournerGauche(0.2,10);
    }
  }
delay(tempsdepause);
}
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
void capteurcouleur(){
   uint16_t clear, red, yellow, blue;
    tcs.setInterrupt(false);      // turn on LED
    delay(60);  // takes 50ms to read
    tcs.getRawData(&red, &yellow, &blue, &clear);
    tcs.setInterrupt(true);  // turn off LED
    Serial.print("C:\t"); Serial.print(clear);
    Serial.print("\tR:\t"); Serial.print(red);
    Serial.print("\tY:\t"); Serial.print(yellow);
    Serial.print("\tB:\t"); Serial.print(blue);
    // Figure out some basic hex code for visualization
    uint32_t sum = clear;
    //float r, g, b;
    r = red; r /= sum;
    y = yellow; y /= sum;
    b = blue; b /= sum;
    r *= 256; y *= 256; b *= 256;
    Serial.print("\t");
    Serial.print((int)r, HEX); Serial.print((int)y, HEX); Serial.print((int)b, HEX);
    Serial.println();
    //Serial.print((int)r ); Serial.print(" "); Serial.print((int)g);Serial.print(" ");  Serial.println((int)b );
    //analogWrite(redpin, gammatable[(int)r]);
    //analogWrite(greenpin, gammatable[(int)g]);
    //analogWrite(bluepin, gammatable[(int)b]);

}
int couleur()
{
  capteurcouleur();
  int valeur=0;
  if (b > r && y > r){
    valeur = 3;
  }
  else if (y > b && r > b){
    valeur = 2;
  }
  else if(r > y && b > y){
    valeur = 1;
  }
  return valeur;
}

/* *********************************************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */
void setup(){
pinMode(A10, INPUT);
BoardInit();
Serial.begin(9600);
   Serial.println("Color View Test!");
    if (tcs.begin()) {
        Serial.println("Found sensor");
    } else {
        Serial.println("No TCS34725 found ... check your connections");
        while (1); // halt!
    }
    // use these three pins to drive an LED
    //pinMode(redpin, OUTPUT);
    //pinMode(greenpin, OUTPUT);
    //pinMode(bluepin, OUTPUT);
    // thanks PhilB for this gamma table!
    // it helps convert RGB colors to what humans see
    for (int i = 0; i < 256; i++) {
        float x = i;
        x /= 255;
        x = pow(x, 2.5);
        x *= 255;
        if (commonAnode) {
            gammatable[i] = 255 - x;
        } else {
            gammatable[i] = x;
        }
        //Serial.println(gammatable[i]);
    }

}

/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

void loop() {

Serial.println(couleur());
delay(1000);

MOTOR_SetSpeed(0,vitesse);
MOTOR_SetSpeed(1,vitesse);

/*if(state){

suiveurdeligne(vitesse,distanceEnPulse(112.5));

avancer(vitesse,distanceEnPulse(112.5));
  
tournerGauche(vitesseTourner,90);
  
avancer(vitesse,distanceEnPulse(71.2));
   
tournerDroite(vitesseTourner,90);
  
avancer(vitesse,distanceEnPulse(73)); //78.2
  
tournerDroite(vitesseTourner,44); //44
  
avancer(vitesse,distanceEnPulse(170)); //166.2
  
tournerGauche(vitesseTourner,87);
  
avancer(vitesse,distanceEnPulse(55));
  
tournerDroite(vitesseTourner,47);
  
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
  

 
}*/

  
}