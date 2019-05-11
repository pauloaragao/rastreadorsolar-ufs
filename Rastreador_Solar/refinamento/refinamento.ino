#include <Arduino.h>
#include <math.h>
#include <DS1307.h>
#include <Wire.h>
#include <MechaQMC5883.h>
#include <MPU6050.h>
#include <Stepper.h>
#include <SPI.h>
#include <SD.h>

int tolerancia = 50;
int ajuste_azim_1 = 0;
int refinamento[3];
int valorSensor_pirel;
int j;

const int stepsPerRevolution_a = 12800;
Stepper myStepper_a(stepsPerRevolution_a, 22, 23, 25, 26);

int sensor_pirel = 9; //pino referente ao pireliômetro
int sensor_piran = 8; //pino referente ao piranômetro

void setup() {
  // put your setup code here, to run once:
   Wire.begin();
  Serial.begin(9600); 

 myStepper_a.setSpeed(1);
 delay(2000);
}

void loop() {
 
     
    valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
    refinamento[0] = map (valorSensor_pirel, 0, 1023, 0, 4000);


     
     //Um passo direita
     for (j = 0; j <= 36; j++) {
        myStepper_a.step(-1); // + 1 GRAU PARA DIREITA
        delay(10);
      }
      delay(2000);
      valorSensor_pirel = 0;
      valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
      refinamento[1] = map (valorSensor_pirel, 0, 1023, 0, 4000);
       
      //Dois passos para esquerda
      for (j = 0; j <= 71; j++) {
        myStepper_a.step(1); // + 1 GRAU PARA ESQUERDA
        delay(10);
      }
      delay(2000);
      valorSensor_pirel = 0;
      valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
      refinamento[2] = map (valorSensor_pirel, 0, 1023, 0, 4000);

      delay(2000);
      //Ajuste central
      for (j = 0; j <= 36; j++) {
        myStepper_a.step(-1); // + 1 GRAU PARA DIREITA
        delay(10);
      }

      //Print dos valoress
      Serial.print("Inicial: "); Serial.print(refinamento[0]);
      Serial.print(" Direita: "); Serial.print(refinamento[1]);
      Serial.print(" Esquerda: "); Serial.println(refinamento[2]);

     delay(10000);

}
