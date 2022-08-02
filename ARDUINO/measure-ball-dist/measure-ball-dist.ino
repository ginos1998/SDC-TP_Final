/*
 HC-SR04 conexiones:
  VCC al arduino 5v 
  GND al arduino GND
  Echo al Arduino pin 3 
  Trig al Arduino pin 2
 */
 
// Definicion de pines
#define echoBall   12 
#define trigBall   11 
#define trigHand   9
#define echo_hand   8
#define pinMotor   10

double maxHeight = 0;  // distancia [cm] del tubo (desde pelota en reposo hasta el sensor) 
const double ballDiam = 6;
double upperGap = 4.5;
int state = 0;
bool flagMotor = false;
const int sampleTime = 20;
int PWM = 150;
double ballPos = 0;
double ballPosFiltered = ballPos;
double ballPosFilt = ballPos;
double alpha = 0.4;
double beta = 0.3;
void setup() {                
  Serial.begin (9600);       // inicializa el puerto seria a 9600 baudios
  pinMode(echoBall, INPUT);     // define el pin 6 como entrada (echo)
  pinMode(trigBall, OUTPUT);    // define el pin 7 como salida  (triger)
  pinMode(pinMotor, OUTPUT);
}
  
void loop() {
  
  if(Serial.available()>1){
    state = Serial.parseInt();
  }
  
  if(state == 1){   
    setUpperGap();
    Serial.println(upperGap);
    state = 0;
  }else if (state == 2){ 
    //Serial.println(ballDist()); 
    Serial.println(initBallPos());
    state = 0;
                                   
  }else if (state == 3){      
    if(flagMotor){
      analogWrite(pinMotor, PWM);
      flagMotor = false;
    }
    Serial.print("ballPos:");
    Serial.print(ballPos);
    Serial.print(",");
    Serial.print("Alpha:");
    Serial.print(ballPosFiltered);
    Serial.print(",");
    Serial.print("Beta:");
    Serial.print(ballPosFilt);
    Serial.println();
    getBallPos();
    delay(sampleTime); 
    
  }else if(state == 4){
    analogWrite(pinMotor, 0);
    state = 0;
    flagMotor = true;
  }

  
}

void setUpperGap(){
  analogWrite(pinMotor, PWM);
  delay(2000);
  double sum = 0;
  upperGap = 0;
  for(int i = 0; i<5; i++){
    sum = sum + ballDist();
    Serial.println(ballDist());
    delay(100);
  }
  Serial.println(sum);
  upperGap = sum/5;
  delay(1000);
  
  
  analogWrite(pinMotor, 0);
}

void getBallPos(){

  ballPos = maxHeight + upperGap - ballDist() - ballDiam;
  
  if(ballPos < 0){
    ballPos = 0;
  }

  ballPosFiltered = alpha*ballPos + (1-alpha)*ballPosFiltered;
  ballPosFilt = beta*ballPos + (1-beta)*ballPosFilt;
}

double initBallPos(){
  
  double ballPos = ballDist() - upperGap + ballDiam;
  if(ballPos < 0){
    ballPos = 0;
  }
  maxHeight = ballPos;
  
  return ballPos;
}

double ballDist(){
  // sensor
  digitalWrite(trigBall, LOW);   // genera el pulso de triger por 10ms
  delayMicroseconds(2);
  digitalWrite(trigBall, HIGH);   // genera el pulso de triger por 10ms
  delayMicroseconds(10);
  digitalWrite(trigBall, LOW);
  
  double duracion = pulseIn(echoBall, HIGH);
  double distancia = duracion * 0.034 / 2;            // calcula la distancia en centimetros

  return distancia;
}
