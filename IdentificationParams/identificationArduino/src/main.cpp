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
#define BUFFERSIZE  2

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
float pos_[BUFFERSIZE];

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

//Vincent
double Trecorded = 0; //garde le temps en millis de la dernière lecture en mémoire
double EnergieTot = 0; //garde l'énergie totale en mémoire
bool Start = false;
bool Arret = false;
bool Restart = false;
bool sendJSON = true;
double Hauteur_foret;
double Distance_bac = 130;
double Distance_arbres = 0; 
double Distance_arbre_robot = 40;
int CAS = -1;
float Lpendule = 37; //en cm
float Hpivot_sol = 94; // en cm
float Hsapin = 9.5; //en cm

bool pretAlacher = false;
float distanceBeak = 20; //cm
float vit = 0;
float breakVit = 0;
float distanceInit;
float vMax = .5;
float vAccel = 0.5;
float vMaxPid = 0.3;
float angle_;
float angle_go;
float encodeurInit = 0;
int state; //machine à états
int stateOscille = 1; //machine à états oscillation
int stateOscillePas = 1;
bool ready_set = false;
float posMin;
float posMax;
float timeStart;

/*------------------------- Prototypes de fonctions -------------------------*/

double Energie();
void timerCallback();
void startPulse();
void endPulse();
void sendMsg(); 
void readMsg();
void serialEvent();
double PIDmeasurementAngle();
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
float absDeg(){return fabs(getAngleDegre())-360*int(fabs(getAngleDegre())/360); }
void setMag(int choix){digitalWrite(MAGPIN, choix);}
void setMoteurs(float v){AX_.setMotorPWM(0,-v);AX_.setMotorPWM(1,-v);}
void stopProgram(){while(true){}}
void resetAngle(){angleInit = -vexEncoderPivot_.getCount();}
void resetEncodeur(){encodeurInit = -vexEncoder_.getCount();}

bool limSwitch(){return analogRead(SWITCH_PIN)>1000;}

void oscilleSIN(float hauteur);
bool oscilleFAT2(float distance, float hauteur); // Pour faire osciller le pendule avec le sapin
bool oscilleFAT3(float distance); // Pour faire osciller le pendule avec le sapin
bool oscillepasFAT2(float distance); // Pour ne pas faire osciller le pendule
void crisseLeCampFAT();
void securiteFAT();
void(* resetFunc) (void) = 0;
void largeurForet();
void normal();
void sapinsMinute();
void sauveEnergie();
void fonctionsProf();
void inputQt();
/*---------------------------- fonctions "Main" -----------------------------*/

void setup() {
  Serial.begin(BAUD);               // initialisation de la communication serielle

  //Serial.println("YOO");


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
  //pidPos_.setGains(0.003,0.00001 ,0);
    // Attache des fonctions de retour
  pidPos_.setMeasurementFunc(getEncoder);
  pidPos_.setCommandFunc(PIDcommand);
  pidPos_.setAtGoalFunc(PIDgoalReached);
  pidPos_.setEpsilon(3);
  pidPos_.setPeriod(100);

  //pidAngle_.setGains(-0.4,-0.00001 ,0);
  pidAngle_.setGains(0.5,0.00001 ,0);

    // Attache des fonctions de retour
  pidAngle_.setMeasurementFunc(PIDmeasurementAngle);
  pidAngle_.setCommandFunc(PIDcommand);
  pidAngle_.setAtGoalFunc(PIDgoalReached);
  pidAngle_.setEpsilon(10);
  pidAngle_.setPeriod(100);

  resetAngle();
  pinMode(MAGPIN,OUTPUT);
  setMag(1);

  pinMode(53,OUTPUT);
  digitalWrite(53,HIGH);
  pinMode(SWITCH_PIN,INPUT);

  delay(1000);
  state = 1;

}

void loop() {
  if(Start){
    switch (CAS)
    {
    case 0:
      normal();
      break;
    
    case 1:
      sapinsMinute();
      break;

    case 2:
      sauveEnergie();
      break;

    case 3:
      largeurForet();
      break;

    default:
      break;
    }
  }

  inputQt();
  fonctionsProf();
}

void securiteFAT() {
  // if (getEncoder() < cmToTick(0) || getEncoder() > cmToTick(120) ){
  //   Serial.println("fuck toi je t'arrêêête");
  //   Stop();
  //   while (true){}
  // }
}



void largeurForet(){
  switch (state)
  {
  
  case 0:
    Stop();
    if(getAngleDegre() > 101) pretAlacher = true;
    if(getAngleDegre()<100 && pretAlacher)setMag(0);
    break;

  case 1:
    setMag(1);

      setMoteurs(-0.15);
      if(limSwitch()){
        setMoteurs(0);
        //resetAngle();
        resetEncodeur();
        delay(1000);
        state = 2;
      }
    break;

  case 2:
    if(distanceTest2(5,.5)) state = 3;
    break;


  case 3:
    oscilleFAT3(4);
    if(getAngleDegre()<-140) ready_set = true;
    if(ready_set){
      if(getAngleDegre() > -90)state = 4;
    }
 
    break;
  
  case 4:
    if(getAngleDegre() > 0)state = 5;
    break;

  case 5:
    setMoteurs(-1);
    delay(75);
    state = 6;
    break;

  case 6:
    if(distanceTest2(Distance_bac-10,1)) state = 0;
    if(getAngleDegre() > 101) pretAlacher = true;
    if(getAngleDegre()<100 && pretAlacher)setMag(0);
    break;

  }

}

void sapinsMinute(){
  switch (state)
  {
    case 1:
      setMag(1);

      setMoteurs(-0.15);
      if(limSwitch()){
        setMoteurs(0);
        resetAngle();
        resetEncodeur();
        delay(1000);
        state = 2;
      } 
      break;
    
    case 2:
      setMoteurs(1);
      while (getEncoder()<cmToTick(38)){}
      setMoteurs(-1);
      delay(175);
      setMoteurs(1);
      delay(200);
      setMag(0);
      delay(200);
      setMoteurs(-1);
      Serial.println("1");
      while(getEncoder()>cmToTick(10)){}
      Serial.println("2");
      setMoteurs(0);
      Serial.println("3");
      delay(500);
      state = 1;
      break;
  }
}

void sauveEnergie(){
  switch (state)
  {
    case 1:
      setMag(1);

      setMoteurs(-0.15);
      if(limSwitch()){
        setMoteurs(0);
        resetAngle();
        resetEncodeur();
        delay(1000);
        state = 2;
      } 
      break;
    
    case 2:
      setMoteurs(1);
      while (getEncoder()<cmToTick(38)){}
      setMoteurs(-1);
      delay(175);
      setMoteurs(1);
      delay(200);
      setMag(0);
      delay(200);
      setMoteurs(-.2);
      Serial.println("1");
      while(getEncoder()>cmToTick(10)){}
      Serial.println("2");
      setMoteurs(0);
      Serial.println("3");
      delay(500);
      state = 1;
      break;
  }
}

void normal(){
  switch (state)
  {
    case 0:
      setMoteurs(0);   
      break;
      
    case 1:
      // recule et attrape le sapin
      setMag(1);

      setMoteurs(-0.15);
      if(limSwitch()){
        setMoteurs(0);
        resetAngle();
        resetEncodeur();
        delay(1000);
        state = 2;
      } 
      break;
    
    case 2:
      // avance et se positionne
      if(distanceTest2(10,vMax)){
     
        stateOscille = 1;  // *** ATTENTION si vous modifiez cette ligne le robot peut crisser le camp par terre et détruire une roue!
        state = 3;
      }
      break;

    case 3:
      // oscille
      if(oscilleFAT2(8,120))state = 4;
      if(getAngleDegre()>360 || getAngleDegre()<-360)state = 1;

      break;

    case 4:
      // avance au panier
      if (distanceTest2(Distance_bac-distanceBeak,1)){
        state = 5;
        setMoteurs(0);
      }
      break;

    case 5:
      // setup PID
      pidAngle_.setGains(-0.4,-0.00001 ,0);
      pidPos_.setGains(0.005,0.00001 ,0);

      pidPos_.setGoal(cmToTick(Distance_bac));
      pidAngle_.setGoal(0);
      pidPos_.enable();
      pidAngle_.enable();

      posMax = getEncoder() + cmToTick(8);
      posMin = getEncoder() + cmToTick(8);
      state = 6;
      timeStart = millis();
      break;

    case 6:

      //stabilise
      pidPos_.run(0);
      pidAngle_.run(1);
      if(millis() > timeStart+10000) state = 7;
      break;

     case 7:
      //lache sapin
      setMag(0);
      setMoteurs(1);
      delay(200);
      setMoteurs(-1);
      delay(200);
      state = 8;
      break;

    case 8:
      //recule vite
      if(distanceTest2(50,.5)) state = 1;
      break;

    default:
      
      
      break;
  }
}

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

bool oscilleFAT2(float distanceMax, float angleDegreGoal){
  
  switch (stateOscille)
  {
  case 1:
    distanceInit = getEncoder();
    angle_  = getAngle();
    // ready_set = false;
    // angle_go = acos((Lpendule-hauteur) / Lpendule);
    stateOscille = 2;
    break;
  
  case 2:
    if(distanceTest2(distanceInit-distanceMax,vMax)) stateOscille = 3;
    break;

  case 3:
    setMoteurs(0);
    if (getAngle() >= angle_ )
      {
        angle_ = getAngle();
      }else
      {
        if( absDeg() < 10) stateOscille = 4;
      }
    break;

  case 4:
    if(distanceTest2(distanceInit + distanceMax,vMax)) stateOscille = 5;
    break;

  case 5:
    setMoteurs(0);
    if (getAngle() <= angle_ )
      {
        angle_ = getAngle();
      }else
      {
        if( absDeg() < 10) stateOscille = 2; 
      }
    break;

    default:
    break;
  }
  
  if(getAngle() < -1*angle_go) ready_set = true;
  // Serial.print("stateOscille = ");
  // Serial.print(stateOscille);
  // Serial.print(" | get_angleDegre() = ");
  // Serial.print(getAngleDegre());
  // Serial.println();
  return getAngleDegre() > angleDegreGoal;

}

bool oscilleFAT3(float distanceMax){
  
  switch (stateOscille)
  {
  case 1:
    distanceInit = getEncoder();
    angle_  = getAngle();
    // ready_set = false;
    // angle_go = acos((Lpendule-hauteur) / Lpendule);
    stateOscille = 3;
    break;
  
  case 2:
    if(distanceTest2(distanceInit-distanceMax,vMax)) stateOscille = 3;
    if(absDeg() > 90) stateOscille = 3;
    break;

  case 3:
    setMoteurs(0);
    if (getAngle() >= angle_ )
      {
        angle_ = getAngle();
      }else
      {
        if( absDeg() < 10) stateOscille = 4;
      }
    break;

  case 4:
    if(distanceTest2(distanceInit + distanceMax,vMax)) stateOscille = 5;
    //if(absDeg() > 90) stateOscille = 5; nerf
    break;

  case 5:
    setMoteurs(0);
    if (getAngle() <= angle_ )
      {
        angle_ = getAngle();
      }else
      {
        if( absDeg() < 10) stateOscille = 2; 
      }
    break;

  }
  //while(getAngleDegre() > 140) setMoteurs(-.8);
  //if(getAngleDegre() < -angleDegreGoal) ready_set = true;

  // Serial.print("stateOscille = ");
  // Serial.print(stateOscille);
  // Serial.print(" | get_angleDegre() = ");
  // Serial.print(getAngleDegre());
  // Serial.println();
  //return ready_set && absDeg() < (angleDegreGoal-10);
  //if(getAngleDegre() < -angleDegreGoal) ready_set = true;
  //return ready_set && getAngleDegre() > angleDegreGoal;
}

//Pour arrêter sec
void Stop(){
    AX_.setMotorPWM(0,0);
    AX_.setMotorPWM(1,0);
}



///////////////////////////////////////////////////////////////////////AFFAIRES DU PROF////////////////////////////////////
// Fonctions pour le PID

void PIDcommand(){
  double cmd = pidAngle_.getCmd();
  

  if ((cmd >= 0 && getEncoder() > posMax ) || (cmd <= 0 && getEncoder() < posMin ))
  {
    cmd = 0;
    //Serial.print("SKEEETCTH :  ");
  }
  
  cmd += pidPos_.getCmd();

  if(cmd > vMaxPid) cmd = vMaxPid;
  else if(cmd < -vMaxPid) cmd = -vMaxPid;

  setMoteurs(cmd);

}

double PIDmeasurementAngle(){
  float cur_pos_ = sin(getAngle());
  for (int i = BUFFERSIZE; i >0; i--)
  {
    pos_[i] = pos_[i-1];
  }
  pos_[0] = cur_pos_;

  float dpos = 0;
  for (int i = 0; i < BUFFERSIZE-1; i++)
  {
    dpos += (pos_[i]-pos_[i+1])/UPDATE_PERIODE;
    // Serial.print(pos_[i]);
    // Serial.print("  ||  ");
  }
  
  float cur_vel_ = dpos/(BUFFERSIZE);

  // Serial.print("  |=|  ");
  // Serial.print(cur_vel_);
  // Serial.println(" ");


  return 1000*cur_vel_;
  //return sin(getAngle());
}


void PIDgoalReached(){
  // To do
  //Serial.println("atGoal !");
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
  doc["Position"] = int (getEncoder()/0.45)/1.71; //(AX_.GetEncoder(0)/3200)*(2*PI*0.06); //en mètre
  doc["StartButton"] = Start;
  doc["StopButton"] = Arret;
  doc["Energie"] = Energie();
  doc["hauteur"] = (Lpendule+Hsapin)*(cos(getAngle())); // AVEC SAPIN. Le 3.978 est un facteur Cambodge
  doc["activateMag"] = activateMag_;
  doc["Reset"] = Restart;

  doc["time"] = millis();
  doc["potVex"] = analogRead(POTPIN);
  // doc["encVex"] = vexEncoder_.getCount();
  doc["goal"] = pidPos_.getGoal();
  doc["motorPos"] = getEncoder();
  doc["voltage"] = AX_.getVoltage();
  doc["current"] = AX_.getCurrent(); 
  doc["pulsePWM"] = pulsePWM_;
  doc["pulseTime"] = pulseTime_;
  // doc["inPulse"] = isInPulse_;
  // doc["accelX"] = imu_.getAccelX();
  // doc["accelY"] = imu_.getAccelY();
  // doc["accelZ"] = imu_.getAccelZ();
  // doc["gyroX"] = imu_.getGyroX();
  // doc["gyroY"] = imu_.getGyroY();
  // doc["gyroZ"] = imu_.getGyroZ();
  //doc["isGoal"] = pid_.isAtGoal();
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
  // Start
    parse_msg = doc["StartButton"];
    if(!parse_msg.isNull()){
        Start = doc["StartButton"];
    }    
  //Reset
   parse_msg = doc["Reset"];
    if(!parse_msg.isNull()){
        Restart = doc["Reset"];
    } 
    //Stop
    parse_msg = doc["StopButton"];
    if(!parse_msg.isNull()){
        Arret = doc["StopButton"];
    }    
    //Hauteur Sapin
    parse_msg = doc["Hauteur_foret"];
    if(!parse_msg.isNull()){
        Hauteur_foret = doc["Hauteur_foret"];
    }    
    //Distance bac
    parse_msg = doc["Distance_bac"];
    if(!parse_msg.isNull()){
        Distance_bac = doc["Distance_bac"];
    } 

    //Distance sapin à sapin
    parse_msg = doc["Distance_arbres"];
    if(!parse_msg.isNull()){
        Distance_arbres = doc["Distance_arbres"];
    }     
    //Distance_arbre_robot
    parse_msg = doc["Distance_arbre_robot"];
    if(!parse_msg.isNull()){
        Distance_arbre_robot = doc["Distance_arbre_robot"];
    }    

    //Distance_arbre_robot
    parse_msg = doc["Choix_defi"];
    if(!parse_msg.isNull()){
        CAS = doc["Choix_defi"];
    }

}

double Energie(){
  EnergieTot +=  AX_.getVoltage()*AX_.getCurrent() * (millis()-Trecorded)/1000;
  Trecorded = millis();
  return EnergieTot;
}

void fonctionsProf(){
   if(shouldRead_){
    readMsg();
  }
  if(shouldSend_ && sendJSON){
    // Serial.print("state = ");
    // Serial.println(state);
    //shouldSend_ = false;
    //Serial.println(Energie());
    sendMsg();
  }
  if(shouldPulse_){
    startPulse();
  }
  // // mise a jour des chronometres
  timerSendMsg_.update();
  // timerPulse_.update();
}

void inputQt(){
  //if(Start) //distanceTest2(10,.2);
  if(Arret) Stop();
  if(Restart) {
    EnergieTot = 0;
    state = 1;
    Restart = false;
    CAS = -1;
  }
}