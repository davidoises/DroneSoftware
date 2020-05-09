/*  
  PRACTICA 1 "SENSOR DE ULTRASONIDOS"
  Muestra la distancia a la que se encuentran los objetos
*/
 
#define pulso 15  //define la salida por donde se manda el pulso como 9
#define rebote 9 //define la salida por donde se recibe el rebote como 10
 
float distancia;  //crea la variable "distancia"
float distancia_previa = 0; 
float tiempo;  //crea la variable tiempo (como float)
 
void setup()
{
  Serial.begin(115200);  //inicializa el puerto serie
  pinMode(pulso, OUTPUT); //Declaramos el pin 9 como salida (pulso ultrasonido)
  pinMode(rebote, INPUT); //Declaramos el pin 8 como entrada (recepción del pulso)
}
 
void loop()
{
  digitalWrite(pulso,LOW); //Por cuestión de estabilización del sensor
  delayMicroseconds(5);
  digitalWrite(pulso, HIGH); //envío del pulso ultrasónico
  delayMicroseconds(10);
  tiempo = pulseIn(rebote, HIGH);  //funcion para medir el tiempo y guardarla en la variable "tiempo"

  distancia_previa = distancia;
  distancia = 0.01715*tiempo; //fórmula para calcular la distancia
  if(tiempo == 0)
  {
    distancia =  distancia_previa;
  }
   
  /*Monitorización en centímetros por el monitor serial*/
  Serial.println(distancia);
  delay(4);
}
