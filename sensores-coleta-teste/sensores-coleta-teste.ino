#include <Wire.h>


int sensor_pirel = 1; //pino referente ao pireliômetro
int sensor_piran = 0; //pino referente ao piranômetro

int valorSensor_pirel, valorSensor_piran, Rad_difusa; 

void setup() {
  // put your setup code here, to run once:

 Wire.begin();
     Serial.begin(9600);
} 
void loop() {
  // put your main code here, to run repeatedly:
int valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
    valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0,4000);
    //PIRANÔMETRO
    int valorSensor_piran = analogRead(sensor_piran); //ler os valores do pireliômetro
    valorSensor_piran = map (valorSensor_piran, 0, 1023, 0,4000);
    //RADIAÇÃO DIFUSA
    delay(100);
    Rad_difusa = valorSensor_piran - valorSensor_pirel ;

    Serial.print ("  Rad Direta: "); Serial.print (valorSensor_pirel); Serial.print (" Rad Global: ");   Serial.print (valorSensor_piran); Serial.print ("  Rad Difusa: ");   Serial.println (Rad_difusa);
    delay(300);
}
