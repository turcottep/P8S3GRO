/* 
 * GRO 302 - Conception d'un robot mobile
 * Code de démarrage
 * Auteurs: Jean-Samuel Lauzon     
 * date: 1 mai 2019
*/

/*------------------------------ Librairies ---------------------------------*/
#include <LibS3GRO.h>
#include <ArduinoJson.h>
#include <libExample.h> // Vos propres librairies
/*------------------------------ Constantes ---------------------------------*/

#define BAUD            115200      // Frequence de transmission serielle
#define UPDATE_PERIODE  100         // Periode (ms) d'envoie d'etat general

#define MAGPIN          32          // Port numerique pour electroaimant
#define POTPIN          A5          // Port analogique pour le potentiometre

#define PASPARTOUR      64          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  50          // Rapport de vitesse du moteur

/*---------------------------- variables globales ---------------------------*/

ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
VexQuadEncoder vexEncoder_;         // objet encodeur vex
IMU9DOF imu_;                       // objet imu
PID pid_;                           // objet PID

volatile bool shouldSend_ = false;  // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;  // drapeau prêt à lire un message
volatile bool shouldPulse_ = true; // drapeau pour effectuer un pulse
volatile bool isInPulse_ = false;   // drapeau pour effectuer un pulse

SoftTimer timerSendMsg_;            // chronometre d'envoie de messages
SoftTimer timerPulse_;              // chronometre pour la duree d'un pulse

uint16_t pulseTime_ = 0;            // temps dun pulse en ms
float pulsePWM_ = 0.3;                // Amplitude de la tension au moteur [-1,1]


float Axyz[3];                      // tableau pour accelerometre
float Gxyz[3];                      // tableau pour giroscope
float Mxyz[3];                      // tableau pour magnetometre

int angleInit;
bool activateMag_ = 0;
float nbr0 = 0;
float nbr1 = 0;
float diff_live;

float p1 = 10;
float p2 = 1;
float p3 = 1;
float p4 = 1;
float p5 = 1;
float p6 = 1;

float vit = 0;
float vMax = 0.15;

/*------------------------- Prototypes de fonctions -------------------------*/

void timerCallback();
void startPulse();
void endPulse();
void sendMsg(); 
void readMsg();
void serialEvent();
double PIDmeasurement();
void PIDcommand(double cmd);
void PIDgoalReached();
void checkSpeed(float diff, float vit_actuelle, int side);
void avance(int distance, int side);
void Stop();
int PoToTic(float cm);
void activateMag(int choix);
void pidPivot(int angle);
/*---------------------------- fonctions "Main" -----------------------------*/

void setup() {
  Serial.begin(BAUD);               // initialisation de la communication serielle
  AX_.init();                       // initialisation de la carte ArduinoX 
  imu_.init();                      // initialisation de la centrale inertielle
  vexEncoder_.init(2,3);            // initialisation de l'encodeur VEX
  // attache de l'interruption pour encodeur vex
  attachInterrupt(vexEncoder_.getPinInt(), []{vexEncoder_.isr();}, FALLING);
  
  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  timerSendMsg_.enable();

  // Chronometre duration pulse
  timerPulse_.setCallback(endPulse);
  
  // Initialisation du PID
  pid_.setGains(0.25,0.1 ,0);
    // Attache des fonctions de retour
    pid_.setMeasurementFunc(PIDmeasurement);
    pid_.setCommandFunc(PIDcommand);
    pid_.setAtGoalFunc(PIDgoalReached);
  pid_.setEpsilon(0.001);
  pid_.setPeriod(10);

  //angleInit = vexEncoder_.getCount;
  pinMode(MAGPIN,OUTPUT);
  
}

void loop() {

 
  avance(PoToTic(20), 0);
  avance(PoToTic(20), 1);
  avance(PoToTic(20), 0);
  avance(PoToTic(20), 1);
  delay(5000);



  // POUR JSON
  if(shouldRead_){
    readMsg();
  }
  if(shouldSend_){
    sendMsg();
  }
  if(shouldPulse_){
    startPulse();
  }
  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();
  
  // mise à jour du PID
  // pid_.run();
}

void pidPivot(int angleGoal, int distanceGoalPO){
  
  float distanceGoal = PoToTic(distanceGoalPO);
  float kpPiv = 0, kiPiv = 0, kdPiv = 0;
  float kpDist = 0, kiDist = 0, kdDist = 0;
  float epsilon = 1.01;
  float angle, distance;
  float erreurAngle, erreurDistance;
  float intErreurAngle = 0, intErreurDistance = 0;
  float derErreurAngle, derErreurDistance;
  float vitesseDist, vitesseAngle;
  bool distanceEstChill = false;
  bool angleEstChill = false;
  float vitesse;

  do{
    //mesure
    distance = (AX_.readEncoder(0) + AX_.readEncoder(1))/2; 
    angle = vexEncoder_.getCount();

    //erreur
    erreurAngle = angleGoal - angle;
    intErreurAngle += erreurAngle;
    derErreurAngle = erreurAngle/UPDATE_PERIODE;

    erreurDistance = distanceGoal - distance;
    intErreurDistance += erreurDistance;
    derErreurDistance = erreurDistance/UPDATE_PERIODE;

    //update
    vitesseDist = kpDist * erreurDistance + kiDist * intErreurDistance + kdDist*derErreurDistance;
    vitesseAngle = kpPiv * erreurAngle + kiPiv * intErreurAngle + kdPiv*derErreurAngle;
    vitesse = min(vMax, vitesseDist + vitesseAngle);
    vitesse = max(0,vitesse);

    AX_.setMotorPWM(0, vitesse); 
    AX_.setMotorPWM(1, vitesse); 

    //check if at goal
    distanceEstChill = distance < distanceGoal*epsilon && distance > distanceGoal/epsilon; 
    angleEstChill = angle < angleGoal*epsilon && angle > angleGoal/epsilon; 

  } while (!distanceEstChill || !angleEstChill);
  
  
}


//NE PAS UTILISER ; c'est utilisé par la fonction avance() pour aller tout droit (aka PID)
void checkSpeed(float diff, float vit_actuelle, int side){
    
  nbr0 =  abs(AX_.readEncoder(0));
  nbr1 =  abs(AX_.readEncoder(0));
  diff_live = nbr0 - nbr1;
  float changement;
    
  if (diff_live < 5) {
    changement = 0.05;
  }else if (diff_live < 10) 
  {
    changement = 0.1;
  }else if (diff_live < 15) 
  {
    changement = 0.15;
  }else if (diff_live < 20) 
  {
    changement = 0.2;
  }
  else
  {
    changement = 0.3;
  }
  
  if (side == 1){
    if ((nbr0 > nbr1)) {
        AX_.setMotorPWM(0, vit_actuelle * (1-changement)); 
        AX_.setMotorPWM(1, vit_actuelle * (1+changement)); 
    } else if ((nbr0 < nbr1)) {
        AX_.setMotorPWM(0, vit_actuelle * (1+changement)); 
        AX_.setMotorPWM(1, vit_actuelle * (1-changement)); 
    } else {
        AX_.setMotorPWM(0,vit_actuelle);
        AX_.setMotorPWM(1,vit_actuelle);
    }
  }else{
     if ((nbr0 > nbr1)) {
        AX_.setMotorPWM(0, vit_actuelle * (1-changement) * -1); 
        AX_.setMotorPWM(1, vit_actuelle * (1+changement) * -1); 
    } else if ((nbr0 < nbr1)) {
        AX_.setMotorPWM(0, vit_actuelle * (1+changement) * -1); 
        AX_.setMotorPWM(1, vit_actuelle * (1-changement) * -1); 
    } else {
        AX_.setMotorPWM(0,vit_actuelle * -1);
        AX_.setMotorPWM(1,vit_actuelle * -1);
    } 
  }
  

  
}

//Pour avancer
void avance(int distance, int side){
  float vitesse = 0.00;
  float enc = 0;
  //Pour acceleration
  float dist_v = 2000;

  abs(AX_.readEncoder(0));
  abs(AX_.readEncoder(1));
  while(abs(AX_.readEncoder(0)) < distance){
    enc = abs(AX_.readEncoder(0));
    if (enc < dist_v) {
      vitesse = 0.05 + ( enc / dist_v) *(vMax - 0.05) ;
    }
    else if ( enc > distance - dist_v) {
      vitesse = 0.05 + ((distance -  enc ) / dist_v) *(vMax - 0.05) ;
    }
    else
    {
      vitesse  = vMax;
    }
    checkSpeed(0,vitesse, side);
  }
  AX_.setMotorPWM(0,0);
  AX_.setMotorPWM(1,0);
  AX_.readResetEncoder(0);
  AX_.readResetEncoder(1);
  //delay(100);
}

//Pour arrêter sec
void Stop(){
    AX_.setMotorPWM(0,0);
    AX_.setMotorPWM(1,0);
    AX_.readResetEncoder(0);
    AX_.readResetEncoder(1);
}

//Pour changer les unites
int PoToTic(float po){
  return (10000*po/(60-14.4));
}

//Pour activer (1) ou desactiver (0) l'aimant
void activateMag(int choix){
    if (choix == 1 || activateMag_){
        digitalWrite(MAGPIN, HIGH); 
    }else{
        digitalWrite(MAGPIN, LOW);
    }
}

// Fonctions pour le PID
double PIDmeasurement(){
  // To do
  return 0;
}
void PIDcommand(double cmd){
  // To do
}
void PIDgoalReached(){
  // To do
}

void startPulse(){
  /* Demarrage d'un pulse */
  timerPulse_.setDelay(pulseTime_);
  timerPulse_.enable();
  timerPulse_.setRepetition(1);
  //AX_.setMotorPWM(0, pulsePWM_);
  //AX_.setMotorPWM(1, pulsePWM_);
  shouldPulse_ = false;
  isInPulse_ = true;
}

void endPulse(){
  /* Rappel du chronometre */
  AX_.setMotorPWM(0,0);
  AX_.setMotorPWM(1,0);
  timerPulse_.disable();
  isInPulse_ = false;
}

void serialEvent(){shouldRead_ = true;}

void timerCallback(){shouldSend_ = true;}

void sendMsg(){
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message

  doc["time"] = millis();
  doc["potVex"] = analogRead(POTPIN);
  doc["encVex"] = vexEncoder_.getCount();
  doc["goal"] = pid_.getGoal();
  doc["motorPos"] = PIDmeasurement();
  doc["voltage"] = AX_.getVoltage();
  doc["current"] = AX_.getCurrent(); 
  doc["pulsePWM"] = pulsePWM_;
  doc["pulseTime"] = pulseTime_;
  doc["inPulse"] = isInPulse_;
  doc["accelX"] = imu_.getAccelX();
  doc["accelY"] = imu_.getAccelY();
  doc["accelZ"] = imu_.getAccelZ();
  doc["gyroX"] = imu_.getGyroX();
  doc["gyroY"] = imu_.getGyroY();
  doc["gyroZ"] = imu_.getGyroZ();
  doc["isGoal"] = pid_.isAtGoal();
  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
  shouldSend_ = false;
}

void readMsg(){
  // Lecture du message Json
  StaticJsonDocument<500> doc;
  JsonVariant parse_msg;

  // Lecture sur le port Seriel
  DeserializationError error = deserializeJson(doc, Serial);
  shouldRead_ = false;

  // Si erreur dans le message
  if (error) {
    Serial.print("deserialize() failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Analyse des éléments du message message

  //MOTEUR
    parse_msg = doc["pulsePWM"];
    if(!parse_msg.isNull()){
      pulsePWM_ = doc["pulsePWM"].as<float>();
    }

    parse_msg = doc["pulseTime"];
    if(!parse_msg.isNull()){
      pulseTime_ = doc["pulseTime"].as<float>();
    }

    parse_msg = doc["pulse"];
    if(!parse_msg.isNull()){
      shouldPulse_ = doc["pulse"];
    }

  //DISTANCES
    parse_msg = doc["p1"];
    if(!parse_msg.isNull()){
      p1 = doc["pulseTime"].as<float>();
    }

    parse_msg = doc["p2"];
    if(!parse_msg.isNull()){
      p2 = doc["pulseTime"].as<float>();
    }

    parse_msg = doc["p3"];
    if(!parse_msg.isNull()){
      p3 = doc["pulseTime"].as<float>();
    }

    parse_msg = doc["p4"];
    if(!parse_msg.isNull()){
      p4 = doc["pulseTime"].as<float>();
    }

    parse_msg = doc["p5"];
    if(!parse_msg.isNull()){
      p5 = doc["pulseTime"].as<float>();
    }

    parse_msg = doc["p6"];
    if(!parse_msg.isNull()){
      p6 = doc["pulseTime"].as<float>();
    }

  // MAGNET
    parse_msg = doc["activateMag"];
    if(!parse_msg.isNull()){
        activateMag_ = doc["activateMag"];
    }
}