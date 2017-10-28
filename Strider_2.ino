#include <ros.h>
#include <strider/Mensaje.h>
#include <geometry_msgs/Twist.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <HMC5883L.h>
#include <TinyGPS.h>


/*************************Definición de variables****************************/

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

//Seccion ROS
ros::NodeHandle nodo;

strider::Mensaje msg;
geometry_msgs::Twist control;
int a,b,c,d,e,f; 

void messageCb(const geometry_msgs::Twist & cmd){
  control.linear.x = cmd.linear.x;
  control.linear.y = cmd.linear.y;
  control.linear.z = cmd.linear.z;
  control.angular.x = cmd.angular.x;
  control.angular.y = cmd.angular.y;
  control.angular.z = cmd.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);
ros::Publisher p("strider", &msg);

//Variables para los sensores de distancia
long distanciaIzq, distanciaDel, distanciaTras, distanciaDcha;
long tiempoIzq, tiempoDel, tiempoTras, tiempoDcha;
long menordistIzq, menordistDel, menordistTras, menordistDcha;

//Definición de pines
int IN3 = 26;    // Con IN3 y IN4 elegimos el sentido de giro del motor
int IN4 = 24;    
int ENB = 10;    // ENB activa o desactiva el motor

//Variables y constantes para la medición de batería
unsigned long Milisactuales; //Variables para hacer posible la medición a intervalos
long Milisanteriores = -1000*10;
long intervalo = 1000*10;
const float BateriaMax = 8; //Voltaje máximo de la batería
int PorVolt;                 
float voltaje = 0.0;         
int nivel;
int zumbador = 46; //Señal de alerta de la batería baja

//Definición de variables varias
char estado;
int modo=0;
int angulo = 90;
int velocidad = 255;
long DistSeg = 50; //Distancia de seguridad para la que la base rectifica la trayectoria
unsigned long Milisactuale; //Variables para hacer posible la medición a intervalos
long Milisanteriore = -100;
long interval = 100;

//Creación del servo del timón
Servo ServoTimon;

//Creación de los servos de barrido
Servo servo;  
Servo servo2;
Servo servo3;
Servo servo4; 
int pos = 90;// Indicador de posición para el servo de barrido

//Variables para la medición de velocidad
float t0=0;
float t1=0;
float aceleracion=0;
float a1=0;
float a2=0;
float v=0;
float i=0;
int calibrar = 0;
float MaxX=-9999,MinX=9999,MaxY=-9999,MinY=9999,MaxZ=-9999,MinZ=9999;
float OffsetX=0,OffsetY=0,OffsetZ=0;

//Variables brújula
HMC5883L compass;
int error = 0;

//Variables GPS
TinyGPS gps;
HardwareSerial & serialgps = Serial1;
unsigned long chars;
unsigned short sentences, failed_checksum;
float latitude, longitude;

/***************************Definición de pines y comunicaciones********************/
void setup() { 
  nodo.initNode();
  nodo.advertise(p);
  nodo.subscribe(sub); 
  
 // Serial.begin(57600);

 //Definición del modo de los pines
   //Pines de los sensores ultrasonidos
     //Sensor izquierdo
     pinMode(35, OUTPUT); 
     pinMode(34, INPUT);

     //Sensor delantero
     pinMode(37, OUTPUT);
     pinMode(36, INPUT); 

     //Sensor  trasero
     pinMode(7, OUTPUT);
     pinMode(6, INPUT);

      //Sensor derecho
     pinMode(9, OUTPUT);
     pinMode(8, INPUT); 

 //Pines del controlador del motor
  pinMode (ENB, OUTPUT); 
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);

 //Pin del servo del timón  
  ServoTimon.attach(42);

 //Pines de los servos de barrido
  servo.attach(30); 
  servo2.attach(31);
  servo3.attach(32);
  servo4.attach(33);

  //Pin del zumbador
  pinMode(zumbador, OUTPUT);

/***********************Acelerómetro********************************************/
    if(!accel.begin())
   {
 /* There was a problem detecting the ADXL345 ... check your connections */
 // Serial.println("Ooops, se ha detectado el ADXL345!");
  while(1);
  }
  sensor_t sensor;
  accel.setRange(ADXL345_RANGE_16_G);/* Seleccionar rango de medición */
  
/*******************Brújula*********************************************************/
Wire.begin(); 
compass = HMC5883L();
error = compass.SetScale(1.3); // Set the scale of the compass.
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
  
  Serial.println("Setting measurement mode to continous.");
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
    
serialgps.begin(9600);
}

/***************************Ejecución del programa principal***************************/
void loop() {
  
   while (calibrar == 0){ //
      sensors_event_t event; 
      accel.getEvent(&event);
  
      for(i=0; i<100; i++){
          if (event.acceleration.x > MaxX) {MaxX = event.acceleration.x;} 
          if (event.acceleration.x < MinX) {MinX = event.acceleration.x;}
      
          if (event.acceleration.y > MaxY) {MaxY = event.acceleration.y;} 
          if (event.acceleration.y < MinY) {MinY = event.acceleration.y;}
      
          if (event.acceleration.z > MaxZ) {MaxZ = event.acceleration.z;} 
          if (event.acceleration.z < MinZ) {MinZ = event.acceleration.z;}
       }
  
      OffsetX = (MaxX + MinX)/2.0;
      OffsetY = (MaxY + MinY)/2.0;
      OffsetZ = (MaxZ + MinZ)/2.0;
    
      calibrar = 1; 
   }
  
  //**************Sensores ultrasonidos*******************//
  //Funciones para medir la distancia en el sensor izquierdo, delantero, trasero y derecho respectivamente.
  if(control.angular.z == 0){
   MedirdistIzq();
   MedirdistDel();
   MedirdistTras();
   MedirdistDcha();
    Milisactuale = millis();  //Almacena los milisegundos actuales
   if(Milisactuales - (Milisanteriore) > (interval)) {  //Comprueba si ha pasado ya el tiempo de intervalo suficiente
      Milisanteriore = Milisactuale; //  
      Brujula();
      MedirVelocidad(OffsetX, OffsetY, OffsetZ);
      ObtenerGPS();
    // Add a time stamp for the message header
       msg.header.stamp = nodo.now();
    // Publish the ROS message 
       p.publish(&msg);  
       nodo.spinOnce();
   }
  }   
  
//***********Servo timon modo automático********************//
while (control.angular.z==1){ 
   MedirdistIzq();
   MedirdistDel();
   MedirdistTras();
   MedirdistDcha();

  if(distanciaDcha<distanciaIzq){
    angulo = 125 - 0.7*(distanciaDcha-DistSeg); 
   if(angulo<90) angulo=90;
   if(angulo>125) angulo=125;
   GirarServo(angulo);
  }
  
   if(distanciaIzq<distanciaDcha){
    angulo = 55 + 0.7*(distanciaIzq-DistSeg); 
     if(angulo<55) angulo=55;
   if(angulo>90) angulo=90;
   GirarServo(angulo);
  }

   if(distanciaDel<50){
    Retroceder();
     if (menordistIzq<menordistDcha){
      GirarDcha();
     }else{
      GirarIzq();
      }
   }
   
   if(distanciaDel>50){
     Avanzar();
   }
      
   Milisactuale = millis();  //Almacena los milisegundos actuales
    if(Milisactuales - (Milisanteriore) > (interval)) {  //Comprueba si ha pasado ya el tiempo de intervalo suficiente
       Milisanteriore = Milisactuale; //
       Brujula();
       MedirVelocidad(OffsetX, OffsetY, OffsetZ);
       ObtenerGPS();
    // Add a time stamp for the message header
       msg.header.stamp = nodo.now();
    // Publish the ROS message 
       p.publish(&msg);  
       nodo.spinOnce();
   }
   MedirBateria();
   delay(10);
 }
  
//Cambia la velocidad de 0 a 4 haciendo un divisior de tensión escribiendo de 0 a 255 en una salida PWM
    if (control.angular.y == 0){
      velocidad=0;}
    if (control.angular.y == 1){
      velocidad=100;}
    if (control.angular.y == 2){
      velocidad=180;}
    if (control.angular.y == 3){
      velocidad=200;}
    if (control.angular.y == 4){
      velocidad=255;}
     
  /***********************Hacia delante**********************/
  //En el estado F movemos el motor para desplazar la base hacia delante 
    if ((control.linear.x == 1) && (control.linear.y == 0) && (control.linear.z == 0) && (control.angular.x == 0)) {
    Avanzar();
    TimonRecto();
    }
  /*****************Hacía delante e izquierda*****************/
  //El estado G es igual que el anterior pero además gira a la izquierda
    else if ((control.linear.x == 1) && (control.linear.y == 0) && (control.linear.z == 0) && (control.angular.x == 1)) {
     Avanzar();
     GirarIzqSuave();
    }
  /******************Hacia delante y derecha*******************/
  //Estado T. Igual que el anterior pero hacia la derecha
    else if ((control.linear.x == 1) && (control.linear.y == 0) && (control.linear.z == 0) && (control.angular.x == -1)) {
      Avanzar(); 
      GirarDchaSuave();
    }
  /***********************Hacia atrás**************************/
  //En el estado B el motor gira en sentido inverso
    else if ((control.linear.x == -1) && (control.linear.y == 0) && (control.linear.z == 0) && (control.angular.x == 0)) {
      Retroceder(); 
      TimonRecto();
    }
  /****************Hacia atrás e izquierda*********************/
  //En el estado H hacemos lo mismo que en anterior pero girando además a la derecha
    else if ((control.linear.x == -1) && (control.linear.y == 0) && (control.linear.z == 0) && (control.angular.x == -1)) {
     Retroceder();
     GirarIzqSuave();
    }
  /***************Hacía detrás y derecha************************/
  //Igual que el anterior pero a la derecha
    else if ((control.linear.x == -1) && (control.linear.y == 0) && (control.linear.z == 0) && (control.angular.x == 1)) {
      Retroceder();
      GirarDchaSuave();
    }
  /**********************Izquierda******************************/
  //Unicamente giro a la izquierda
    else if ((control.linear.x == 0) && (control.linear.y == 0) && (control.linear.z == 0) && (control.angular.x == 1)) {
     GirarIzq();
     Avanzar();
    }
  /***********************Derecha*******************************/
  //Unicamente giro a la derecha
    else if ((control.linear.x == 0) && (control.linear.y == 0) && (control.linear.z == 0) && (control.angular.x == -1)) {
     GirarDcha();
     Avanzar();
    }
  /************************Parada*****************************/
  //En este estado se para el motor
    else if ((control.linear.x == 0) && (control.linear.y == 0) && (control.linear.z == 0) && (control.angular.x == 0)) {
      analogWrite(ENB,0);      
    }
    
  MedirBateria();
 //Para el siguiente bucle
 delay(100);
}

long MedirdistIzq(){
  digitalWrite(35, LOW); //Estabilización de sensores
  delayMicroseconds(5);
  digitalWrite(35, HIGH);  // Envío del pulso ultrasónico
  delayMicroseconds(10);
  tiempoIzq = pulseIn(34, HIGH); // Función para medir la longitud del pulso entrante Izq.
  distanciaIzq = int(0.017 * tiempoIzq); // Fórmula para calcular la distancia Izq obteniendo un valor entero
  msg.dist.g = distanciaIzq;
  return distanciaIzq;
}

long MedirdistDel(){
  digitalWrite(37, LOW);
  delayMicroseconds(5);
  digitalWrite(37, HIGH);
  delayMicroseconds(10);
  tiempoDel = pulseIn(36, HIGH);
  distanciaDel = int(0.017 * tiempoDel);
  msg.dist.r = distanciaDel;
  return distanciaDel;
}

long MedirdistTras(){
  digitalWrite(7, LOW);
  delayMicroseconds(5);
  digitalWrite(7, HIGH);
  delayMicroseconds(10);
  tiempoTras = pulseIn(6, HIGH);
  distanciaTras = int(0.017 * tiempoTras);
  msg.dist.a = distanciaTras;
  return distanciaTras;
}

long MedirdistDcha(){
  digitalWrite(9, LOW);
  delayMicroseconds(5);
  digitalWrite(9, HIGH);
  delayMicroseconds(10);
  tiempoDcha = pulseIn(8, HIGH);
  distanciaDcha = int(0.017 * tiempoDcha);
  msg.dist.b = distanciaDcha;
  return distanciaDcha;
}

void Avanzar(){
digitalWrite (IN3, HIGH);
digitalWrite (IN4, LOW);
analogWrite(ENB,velocidad); 
}

void AvanzarProp(int velocidad){
digitalWrite (IN3, HIGH);
digitalWrite (IN4, LOW);
analogWrite(ENB,velocidad); 
}

void Retroceder(){
digitalWrite (IN3, LOW);
digitalWrite (IN4, HIGH);
analogWrite(ENB,velocidad); 
}

void MedirBateria(){
 /*************************Medidor de batería***********************/
    Milisactuales = millis();  //Almacena los milisegundos actuales
    if(Milisactuales - (Milisanteriores) > (intervalo)) {  //Comprueba si ha pasado ya el tiempo de intervalo suficiente
       Milisanteriores = Milisactuales; //Guarda el almacenaje como anteriores para la siguiente pasada
       voltaje = (analogRead(A15)*5.015 / 1024.0)*11.132; //Lee el voltaje del pin A13 y realiza las operaciones para convertirlo a voltios
       PorVolt = (voltaje*100)/ BateriaMax; //Calcula el porcentaje y muestra un nivel de carga en función del resultado obtenido
       if      (PorVolt<=75)               { nivel=0; digitalWrite(zumbador,HIGH);} //Cuando la bateria está baja se activa el zumbador
       else if (PorVolt>75 && PorVolt<=80) { nivel=1; }    
       else if (PorVolt>80 && PorVolt<=85) { nivel=2; }    
       else if (PorVolt>85 && PorVolt<=90) { nivel=3; }    
       else if (PorVolt>90 && PorVolt<=95) { nivel=4; }    
       else if (PorVolt>95)                { nivel=5; }   
       msg.volt = voltaje;  //Imprime el voltaje de la batería
    }  
}

void MedirVelocidad(float OffsetX, float OffsetY, float OffsetZ){
  sensors_event_t event; //Medir aceleraciones
  accel.getEvent(&event);

 //Sumadas o restadas los valores de offset para poner a cero la medición en estático 
  msg.acel.x = event.acceleration.x + 0.35;
  msg.acel.y = event.acceleration.y - OffsetY;
  msg.acel.z = event.acceleration.z - 9.45;
   
  a1=event.acceleration.x - OffsetX;
  a2=event.acceleration.y - OffsetY;
  aceleracion=sqrt(pow(a1,2) + pow(a2,2));
  t1=millis()/1000.00;
  if ((abs(a1)>0.08 || abs(a2)>0.08)){ 
     v=v+0.5*aceleracion*(t1-t0);
  }else{
    aceleracion=0;
    v=0;
    }
  t0=t1;

  msg.vel = v;
}

void GirarServo(int angulo){
angulo = constrain(angulo, 55, 125); //restringimos el valor de 55 a 125
ServoTimon.write(angulo); //Escritura del valor del ángulo en la salida del servo
msg.ang = angulo;
}

void TimonRecto(){
angulo = 90; 
ServoTimon.write(angulo);
msg.ang = angulo;
}

void GirarIzqSuave(){
angulo = 105; 
ServoTimon.write(angulo);
msg.ang = angulo;
}

void GirarDchaSuave(){
angulo = 75; 
ServoTimon.write(angulo); 
msg.ang = angulo;
}

void GirarIzq(){
angulo = 125; 
ServoTimon.write(angulo); 
msg.ang = angulo;
}

void GirarDcha(){
angulo = 55; 
ServoTimon.write(angulo); 
msg.ang = angulo;
}

void Brujula(){
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// 
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  float declinationAngle = 0.089;
  heading += declinationAngle;
  
  if(heading < 0)
    heading += 2*PI;
    
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convertir radianes a grados
  float headingDegrees = heading * 180/M_PI; 
  
  msg.bruj = headingDegrees;
}

void ObtenerGPS(){  
  while(serialgps.available()){
     int c = serialgps.read(); 
     if(gps.encode(c)){
        gps.f_get_position(&latitude, &longitude);
        Serial.print("Latitud/Longitud: "); 
        Serial.print(latitude,5); 
        Serial.print(", "); 
        Serial.println(longitude,5);
        Serial.print("Altitud (metros): "); Serial.println(gps.f_altitude()); 
        Serial.println();
        gps.stats(&chars, &sentences, &failed_checksum);
      }
  }
 
  msg.lati = latitude;
  msg.longi = longitude;
  msg.alti = gps.f_altitude();
}
