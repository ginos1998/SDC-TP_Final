 /*   
 *  Control de levitacion de pelota en tubo cilindrico vertical
 *  
 *  Desarrollado por SIAMPICHETTI Gino, MAFFINI Agustin
 *  Facultad de Ciencias Exactas, Fisicas y Naturales (FCEFyN, UNC)
 *  Asignatura Sistemas de Control 2021
 *  Proyecto Integrador
 *  Documentacion relacionada : *link *
 */

/*****************************************************************************************************************************/
#include <PID_v1.h>

// Definicion de pines
#define pinEchoBall   12 
#define pinTrigBall   11 
#define pinTrigHand   9
#define pinEchoHand   8
#define pinMotor      10

/***************************************** Variables para sensores ultrasonicos **********************************************/
// sensor de la pelota
double ballDuration;
double ballDist = 0;
double prevBallPos = 0;
double ballFiltered = 0;
double alpha = 0.3;
int ballSampleTime = 24;
int ballPrevTime = 0;

// sensor de la mano
double handDuration;
double handDist;
double  prevHandPos = 0; 
double handFiltered = 0;
double beta = 0.1;
int handSampleTime = 100;
int handPrevTime = 0;

/***************************************** Variables del sistema Fisico ******************************************************/
double  lowerGap = 6.0;
double  upperGap = 4.0;
double  ballDiam = 6.0;
double  maxLenght = 45.5;
const double regA = 9.0;
const double regB = 18.0;
const double regC = 30.0;
const double regD = 40.0;
char region = 'a';  // La pelota parte del reposo
double output; 
double setPoint = 22;                

/***************************************** Variables de Software ************************************************************/

int state = 0;

// Variables puerto serie
int prevTimeSerial = 0;
int elapsedTimeSerial = 0;
int serialSampleTime = 100;

// Variables PID
int PIDsampleTime = 50;

// listo 
double Kpa = 4.3525;         // +20%    
double Kia = 4.8;             // -27%
double Kda = 0.1;            // -10% 
// listo
double Kpb = 3.4821;         // +10%    
double Kib = 5.6;             // -27%
double Kdb = 0.2;            // -10% 
// listo
double Kpc = 3.3481;    // +0%
double Kic = 5.2;    // -10%
double Kdc = 0.4;     // -22.5%
// 
double Kpd = 1.25;    // -10%
double Kid = 1.0;   // -15%
double Kdd = 0.01;       // -50%

/*
// testeado 15/7/22 15:58 
double Kpa = 4.3525;         // +20%    
double Kia = 4.6;             // -27%
double Kda = 0.1;            // -10% 
// testeado 18/7/22 08:39 
double Kpb = 3.3481;    // +0%
double Kib = 5.2;    // -10%
double Kdb = 0.5;     // -22.5%
// testeado 18/7/22 10:39
double Kpc = 1.5;    // -10%
double Kic = 1.25;   // -15%
double Kdc = 0.075;       // -50%
*/
/*
// testeado 15/7/22 15:58 
double Kpa = 4.01772;         // +20%    
double Kia = 4.6;             // -27%
double Kda = 0.02;            // -98% 
// testeado 18/7/22 08:39 
double Kpb = 3.3481;    // +0%
double Kib = 5.3708;    // -10%
double Kdb = 0.225;     // -22.5%
// testeado 18/7/22 10:39
double Kpc = 3.0133;    // -10%
double Kic = 5.07246;   // -15%
double Kdc = 0.5;       // -50%
*/

PID myPID(&ballFiltered, &output, &setPoint, Kpa, Kia, Kda, DIRECT);

/***************************************** Configuraciones iniciales *********************************************************/

void setup() {     
  Serial.begin (9600);          // inicializa el puerto seria a 9600 baudios
  pinMode(pinEchoBall,  INPUT);    // define el pin 12 como entrada (echo)
  pinMode(pinTrigBall,  OUTPUT);   // define el pin 11 como output~ (triger)
  pinMode(pinEchoHand, INPUT);    // define el pin 4 como entrada (echo)
  pinMode(pinTrigHand,  OUTPUT);   // define el pin 5 como output~ (triger)
  pinMode(pinMotor,  OUTPUT);   // define el pin 3 como output~ (motor)
  init_values();                // cargo valores inciales
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 150.0);
  myPID.SetSampleTime(PIDsampleTime);
  delay(500);                 
}

/*****************************************************************************************************************************/

void loop() {

  if(Serial.available()>1){
    state = Serial.parseInt();
  }
  
  if (state==1){
    while(state==1){

      ball_sensor();
      hand_sensor();

      if(region == 'a'){
        myPID.SetTunings(Kpa, Kia, Kda);
      }else if(region == 'b'){
        myPID.SetTunings(Kpb, Kib, Kdb);
      }else if(region == 'c'){
        myPID.SetTunings(Kpc, Kic, Kdc);
      }else if(region == 'd'){
        myPID.SetTunings(Kpd, Kid, Kdd);
      }else{
        Serial.println("Invalid region");
      }
  
      myPID.Compute();
      analogWrite(pinMotor, output);

      printSerial();
    
      delay(1);

      if(Serial.available()>1){
        state = Serial.parseInt();
      }
    }
  
  }else if(state == 2){
    init_values();
    state = 0;
  }
  
}

/*****************************************************************************************************************************/

void ball_sensor(){
  int currentTime = (int) millis();
  int elapsedTime = currentTime - ballPrevTime;
  
  if(elapsedTime >= ballSampleTime){
    
    digitalWrite(pinTrigBall, LOW);     
    delayMicroseconds(2);
    digitalWrite(pinTrigBall, HIGH);                // genera el pulso de triger por 10us
    delayMicroseconds(10);
    digitalWrite(pinTrigBall, LOW);     
  
    ballDuration = pulseIn(pinEchoBall, HIGH);
    ballDist = ballDuration * 0.034 / 2;        // calcula la distancia medida de la pelota, en centimetros

    // calculo la distancia de elevacion de la pelota, en tiempo real, tomando su punto medio como referencia
    ballDist = maxLenght + upperGap - ballDist - ballDiam;
    // filtro pasabajos (filtro media movil exponencial) para atenuar el ruido
    ballFiltered = alpha*ballDist + (1-alpha)*ballFiltered;
    
    // manejo posibles errores del sensor
    if(ballFiltered < 0.0)  ballFiltered = 0.0;
     
    // si el sensor detecta una distancia mayor a regC, es un error. Entonces, tomo el valor anterior.
    if(ballFiltered > regD){
      ballFiltered = regD;             
      prevBallPos = regD;            
    }else{
      setRegion();
    }

    ballPrevTime = currentTime;
  }
}

/*****************************************************************************************************************************/

void setRegion(){
  if(ballFiltered <= regA){
    region = 'a';
  }else if(ballFiltered > regA && ballFiltered <= regB){
    region = 'b';
  }else if(ballFiltered > regB && ballFiltered < regC){
    region = 'c';
  }else if(ballFiltered > regC && ballFiltered < regD){
    region = 'd';
  }
}

/*****************************************************************************************************************************/

void hand_sensor(){
  int currentTime = (int) millis();
  int elapsedTime = currentTime - handPrevTime;

  if(elapsedTime >= handSampleTime){
    
    digitalWrite(pinTrigHand, LOW);   
    delayMicroseconds(2);
    digitalWrite(pinTrigHand, HIGH);              // genera el pulso de triger por 10ms
    delayMicroseconds(10);
    digitalWrite(pinTrigHand, LOW);
    handDuration = pulseIn(pinEchoHand, HIGH);
    
    // calcula la distancia de la mano, en centimetros, -lowerGap por la posicion fisica del sensor
    handDist = (handDuration * 0.034 / 2) - lowerGap;      
    //
    handFiltered = beta*handDist + (1-beta)*handFiltered;

    if(handFiltered < 0) handFiltered = 0;
    
    /*
     * como el sensor mide desde 3cm hasta 3m, 
     * establecemos que si en el rango del tubo no hay referencia de la mano,
     * el setPoint es la ultima referencia tomada
    */
    if(handFiltered > regD){
      handFiltered = prevHandPos;
    }else{
      prevHandPos = handFiltered;
    }
    
    setPoint = handFiltered;

    handPrevTime = currentTime;
  }                    
}

/*****************************************************************************************************************************/

void init_values(){
  prevHandPos = setPoint;
  analogWrite(pinMotor, 0);
  Serial.flush();
}

/*****************************************************************************************************************************/

void printSerial(){
  int currentTime = (int)millis();
  int elapsedTime = currentTime - prevTimeSerial;

  // imprimir Serial Plotter
  if(elapsedTime >= serialSampleTime){
    Serial.print("Pelota:");
    Serial.print(ballFiltered);
    Serial.print(",");
    Serial.print("Ref:");
    Serial.print(setPoint);
    Serial.print(",");
    Serial.print("MaxLenght:");
    Serial.print(regD);
    Serial.println();

    prevTimeSerial = currentTime;
  }
}

/*****************************************************************************************************************************/
