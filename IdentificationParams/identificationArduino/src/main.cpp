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
#define SWITCH_PIN      8

#define POTPIN          A5          // Port analogique pour le potentiometre

#define PASPARTOUR      64          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  50          // Rapport de vitesse du moteur

/*---------------------------- variables globales ---------------------------*/

ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
VexQuadEncoder vexEncoder_;         // objet encodeur vex
VexQuadEncoder vexEncoderPivot_;         // objet encodeur vex

IMU9DOF imu_;                       // objet imu
PID pidPos_;                           // objet PID
PID pidAngle_;                           // objet PID

volatile bool shouldSend_ = false;  // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;  // drapeau prêt à lire un message
volatile bool shouldPulse_ = true; // drapeau pour effectuer un pulse
volatile bool isInPulse_ = false;   // drapeau pour effectuer un pulse

SoftTimer timerSendMsg_;            // chronometre d'envoie de messages
SoftTimer timerPulse_;              // chronometre pour la duree d'un pulse

uint16_t pulseTime_ = 0;            // temps dun pulse en ms
float pulsePWM_ = 0.7;                // Amplitude de la tension au moteur [-1,1]


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
float breakVit = 0;
float distanceInit;
float vMax = .4;
float angle_;
float angle_go;
float longueur = 69;
float encodeurInit = 0;
int state = 1; //machine à états
int stateOscille = 1; //machine à états oscillation
int stateOscillePas = 1;
bool ready_set = false;

/*------------------------- Prototypes de fonctions -------------------------*/

void timerCallback();
void startPulse();
void endPulse();
void sendMsg(); 
void readMsg();
void serialEvent();
double PIDmeasurement();
void PIDcommand();
void PIDgoalReached();
void checkSpeed(float diff, float vit_actuelle, int side);
void avance(int distance, int side);
void Stop();
int PoToTic(float cm);
void pidPivot(int angle);
void distanceTest(float distanceGoal);
bool distanceTest2(float distanceGoal, float vit);
double getEncoder(){return -vexEncoder_.getCount() - encodeurInit;}
double getAngle(){return float(-vexEncoderPivot_.getCount() - angleInit)/88*2*PI;}
double getAngleDegre(){return getAngle()/(2*PI)*360;}
float cmToTick(float d){return d/0.425;}
void activateMag(int choix){digitalWrite(MAGPIN, choix);}
void setMoteurs(float v){AX_.setMotorPWM(0,-v);AX_.setMotorPWM(1,-v);}
void stopProgram(){while(true){}}
void resetAngle(){angleInit = -vexEncoderPivot_.getCount();}
void resetEncodeur(){encodeurInit = -vexEncoder_.getCount();}

bool limSwitch(){return analogRead(SWITCH_PIN)>1000;}

void oscilleSIN(float hauteur);
bool oscilleFAT2(float distance, float hauteur); // Pour faire osciller le pendule avec le sapin
bool oscillepasFAT2(float distance); // Pour ne pas faire osciller le pendule
void crisseLeCampFAT();
void securiteFAT();

/*---------------------------- fonctions "Main" -----------------------------*/

void setup() {
  Serial.begin(BAUD);               // initialisation de la communication serielle

  Serial.println("YOO");


  AX_.init();                       // initialisation de la carte ArduinoX 
  imu_.init();                      // initialisation de la centrale inertielle
  vexEncoder_.init(2,3);            // initialisation de l'encodeur VEX
  vexEncoderPivot_.init(18,19);            // initialisation de l'encodeur VEX

  // attache de l'interruption pour encodeur vex
  attachInterrupt(vexEncoder_.getPinInt(), []{vexEncoder_.isr();}, FALLING);
  attachInterrupt(vexEncoderPivot_.getPinInt(), []{vexEncoderPivot_.isr();}, FALLING);

  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  timerSendMsg_.enable();

  // Chronometre duration pulse
  timerPulse_.setCallback(endPulse);
  
  // Initialisation du PID
  // pidPos_.setGains(0.001,0 ,0);
  //   // Attache des fonctions de retour
  // pidPos_.setMeasurementFunc(getEncoder);
  // pidPos_.setCommandFunc(PIDcommand);
  // pidPos_.setAtGoalFunc(PIDgoalReached);
  // pidPos_.setEpsilon(10);
  // pidPos_.setPeriod(100);

  // pidAngle_.setGains(-0.01,0 ,0);
  //   // Attache des fonctions de retour
  // pidAngle_.setMeasurementFunc(getAngleDegre);
  // pidAngle_.setCommandFunc(PIDcommand);
  // pidAngle_.setAtGoalFunc(PIDgoalReached);
  // pidAngle_.setEpsilon(10);
  // pidAngle_.setPeriod(100);

  resetAngle();
  pinMode(MAGPIN,OUTPUT);
  activateMag(1);

  pinMode(53,OUTPUT);
  digitalWrite(53,HIGH);
  pinMode(SWITCH_PIN,INPUT);

  delay(1000);
  // distanceTest2(-20, .1);
  // distanceTest2(0, .1);
}

void loop() {
  
  // Serial.print("state = ");
  // Serial.println(state);

  switch (state)
  {
    case 0:
      setMoteurs(0);
      break;
      
    case 1:
      // recule et attrape le sapin
      setMoteurs(-0.1);
      if(limSwitch()){
        setMoteurs(0);
        activateMag(1);
        resetAngle();
        resetEncodeur();
        delay(1000);
        state = 2;
      } 
      break;
    
    case 2:
      // avance et se positionne
      if(distanceTest2(10,vMax))state = 3;
      stateOscille = 1;  // *** ATTENTION si vous modifiez cette ligne le robot peut crisser le camp par terre et détruire une roue!
      break;

    case 3:
      // oscille
      if(oscilleFAT2(2,140))state = 4;
      break;

    case 4:
      // avance au panier
      if (distanceTest2(120,.8))state = 5;
      break;

    case 5:
      setMoteurs(0);
      // while (1){
      //   double angle_pendule = getAngle();
      //   if (angle_pendule < -90 && angle_pendule > -70){
      //     if (getAngle() < -70 && getAngle() > -50){
      //       activateMag(0);
      //       break;
      //     }
      //   }
      // }
      activateMag(0);
        state = 6;
      } 
      //setup PID
      // pidPos_.setGoal(cmToTick(100));
      // pidAngle_.setGoal(0);
      // pidPos_.enable();
      // pidAngle_.enable();
      // state = 6;
      break;

    case 6:
      setMoteurs(-0.1);
      if(limSwitch()) setMoteur(0);
      state = 0;
      //stabilise
      // pidPos_.run(0);
      // pidAngle_.run(1);
      // if (pidPos_.isAtGoal() && pidAngle_.isAtGoal())  state = 0;
      break;

    default:
      
      
      break;
  }
  
  // // POUR JSON
  // if(shouldRead_){
  //   readMsg();
  // }
  // if(shouldSend_){
  //   sendMsg();
  // }
  // if(shouldPulse_){
  //   startPulse();
  // }
  // // mise a jour des chronometres
  // timerSendMsg_.update();
  // timerPulse_.update();
    
}

void securiteFAT() {
  // if (getEncoder() < cmToTick(0) || getEncoder() > cmToTick(120) ){
  //   Serial.println("fuck toi je t'arrêêête");
  //   Stop();
  //   while (true){}
  // }
}

void crisseLeCampFAT(){
  do
  {
    securiteFAT();
    AX_.setMotorPWM(0,-vMax);
    AX_.setMotorPWM(1,-vMax);

  } while (true);
}

bool oscilleFAT2(float distanceMax, float angleDegre){
  
  switch (stateOscille)
  {
  case 1:
    distanceInit = getEncoder();
    angle_  = getAngle();
    // ready_set = false;
    // angle_go = acos((longueur-hauteur) / longueur);
    stateOscille = 2;
    break;
  
  case 2:
    if(distanceTest2(distanceInit + distanceMax,vMax)) stateOscille = 3;
    break;

  case 3:
    setMoteurs(0);
    if (getAngle() >= angle_ )
      {
        angle_ = getAngle();
      }else
      {
        stateOscille = 4;
      }
    break;

  case 4:
    if(distanceTest2(distanceInit,vMax)) stateOscille = 5;
    break;

  case 5:
    setMoteurs(0);
    if (getAngle() <= angle_ )
      {
        angle_ = getAngle();
      }else
      {
        if (angle_ >= 90) delay(1250);
        stateOscille = 2;
      }
    break;

    default:
    break;
  }
  
  if(getAngle() < -1*angle_go) ready_set = true;
  Serial.print("stateOscille = ");
  Serial.print(stateOscille);
  Serial.print(" | get_angleDegre() = ");
  Serial.print(getAngleDegre());
  Serial.print(" | angle_ = ");
  Serial.print(angle_);
  Serial.print(" | angle_go = ");
  Serial.print(angle_go);
  Serial.print(" | ready:");
  Serial.println(ready_set);
  return getAngleDegre() > angleDegre;

}

// bool oscillepasFAT2(float distance){
  
//   switch (stateOscillePas)
//   {
//     case 1:
//     SetMoteur(0);
//     break;

//     case 2:
//     SetMoteur(0);
//     break;
    
//     default:
//     break;
//   }
// }


bool distanceTest2(float distanceCM, float vit){
  float distanceGoal = cmToTick(distanceCM);
  int sens;
  int distInit = getEncoder();
  float distance;
  float v = vit;

  if (distanceGoal < distInit) sens = 1;
  else sens = -1;

  AX_.setMotorPWM(0,sens * v);
  AX_.setMotorPWM(1,sens * v);
  // do
  // {
    securiteFAT();
    distance = getEncoder();
  //   // Serial.print(distanceGoal);
  //   // Serial.print("  ||  ");
  //   // Serial.println(distance);
  // } while (distance < distanceGoal - 1 || distance > distanceGoal + 1);

  return !(distance < distanceGoal - 1 || distance > distanceGoal + 1);
  AX_.setMotorPWM(0,-sens * breakVit);
  AX_.setMotorPWM(1,-sens * breakVit);

}




//Pour arrêter sec
void Stop(){
    AX_.setMotorPWM(0,0);
    AX_.setMotorPWM(1,0);
}



///////////////////////////////////////////////////////////////////////AFFAIRES DU PROF////////////////////////////////////
// Fonctions pour le PID

void PIDcommand(){
  // double cmd = pidAngle_.getCmd();
  // cmd += 0.7 * pidPos_.getCmd();
  // setMoteurs(cmd);
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
  doc["goal"] = pidPos_.getGoal();
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
  doc["isGoal"] = pidPos_.isAtGoal();
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