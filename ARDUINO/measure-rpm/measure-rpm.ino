// ********* variables *********

int motor_pin = 3;
int pin_button = 4;
int motor_state = HIGH;
int RPM = 0;              // variable para guardar las RPM
volatile int contador = 0;// Variable entera que se almacena en la RAM del Micro
int actual_time = 0;      // variable para almacenar el tiempo actual durante la ejecucion
int n_aspas = 7;
int pin_sensor = 2;

const long sampleTime = 50, tau = 500; // tiempo de muestreo, periodo senal salida
unsigned long lastTime = 0, prev_time = 0; 

void setup() {
  pinMode (motor_pin, OUTPUT);  // pin6: salida para controlar motor
  pinMode(pin_button, INPUT);
  attachInterrupt(0, sensor_rpm, RISING);  // Habilito Interrupcion 0 (pin2), accion, modo interrupcion
  Serial.begin(9600);   // velocidad puerto serial
  delay(2000);
  digitalWrite(motor_pin, HIGH);
}

void loop() {

  actual_time = millis() - 2000; // almaceno el tiempo actual de ejecucion
  
  if(actual_time-lastTime >= sampleTime){  // verifico que haya transcurrido un periodo de muestreo para realizar otro
    
    noInterrupts();         // desconecto la interrupción para que no actué en esta parte del programa.
    RPM = 0.3*((contador*60000.0)/((actual_time-lastTime)*n_aspas)) + 0.7*RPM;  // calculo las revoluciones por minuto
    //RPM = ((contador*60000.0)/((actual_time-lastTime)*n_aspas));              // el filtro esta para eliminar impreciosiones del sensor
    lastTime = actual_time; // almaceno el tiempo actual
    contador = 0;           // reinicio contador para volver a calcular en el siguiente muestreo
    interrupts();           // Restart the interrupt processing // Reiniciamos la interrupción
  
    //read_input();

    char datos [16];
    sprintf(datos, "%d %d %d", actual_time, RPM, motor_state);
    Serial.println(datos);
    
  }

  //square_signal();

  if(actual_time >= 6000){
    digitalWrite(motor_pin, LOW);
  }
}
  

void sensor_rpm(){     // Funcion que se ejecuta durante cada interrupion
  if(digitalRead(pin_sensor)){
    contador++;             // Se incrementa en uno el contador
  }
}

void read_input(){
  motor_state = digitalRead(pin_button);

  if(motor_state == LOW){
    digitalWrite(motor_pin, HIGH);
  }else{
    digitalWrite(motor_pin, LOW);
  }
}

//void square_signal(){
//  
//  if(actual_time - prev_time >= tau){
//    prev_time = actual_time;
//    
//    if(motor_state == LOW){
//      motor_state = HIGH;
//    }else{
//      motor_state = LOW;
//    }
//    
//    digitalWrite(motor_pin, motor_state);
//    Serial.println(5);
//    
//    //Serial.println(" RPM  " + RPM);
//  }
//}
