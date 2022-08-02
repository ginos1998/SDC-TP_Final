#define pinTrigHand   9
#define pinEchoHand   8

double handDuration = 0;
double handDist = 0;
double lowerGap = 6.0;
double prevHandPos = 0;
double setPoint = 0;
double setPointFiltered = 0;
double alpha = 0.1;

int currentTime = 0;
int prevTimeSerial = 0;
int elapsedTimeSerial = 0;
int prevTimeSensor = 0;
int elapsedTimeSensor = 0;
int serialSampleTime = 500;
int sensorSampleTime = 100;

void setup() {
  // put your setup code here, to run once:
  Serial.begin (9600);
  pinMode(pinEchoHand, INPUT);      // define el pin 4 como entrada (echo)
  pinMode(pinTrigHand,  OUTPUT);    // define el pin 5 como output~ (triger)
}

void loop() {
  hand_sensor();
  printSerial();
  delay(100);
}

void hand_sensor(){
  currentTime = (int)millis();
  elapsedTimeSensor = currentTime - prevTimeSensor;

  if(elapsedTimeSensor >= sensorSampleTime){
    prevTimeSensor = currentTime;

    digitalWrite(pinTrigHand, LOW);   
    delayMicroseconds(2);
    digitalWrite(pinTrigHand, HIGH);              // genera el pulso de triger por 10ms
    delayMicroseconds(10);
    digitalWrite(pinTrigHand, LOW);
  
    handDuration = pulseIn(pinEchoHand, HIGH);
    handDist = handDuration * 0.034 / 2;            

    setPoint = handDist - lowerGap;                    
    setPointFiltered = alpha*setPoint + (1-alpha)*setPointFiltered;
    /*
     * como el sensor mide desde 3cm hasta 3m, 
     * establecemos que si en el rango del tubo no hay referencia de la mano,
     * el setPoint es la ultima referencia tomada
     */
    if(setPoint > 40 || setPointFiltered > 40){                                  
      setPoint = prevHandPos;
      setPointFiltered = prevHandPos;                      
    }else{
      prevHandPos = setPoint;
    }
  }
}

void printSerial(){
  currentTime = (int)millis();
  elapsedTimeSerial = currentTime - prevTimeSerial;

  // imprimir Serial Plotter
  if(elapsedTimeSerial >= serialSampleTime){
    Serial.print("Ref:");
    Serial.print(setPoint);
    Serial.print(",");
    Serial.print("RefFiltered:");
    Serial.print(setPointFiltered);
    Serial.println();
    prevTimeSerial = currentTime;
  }
}
