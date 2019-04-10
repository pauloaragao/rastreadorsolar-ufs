// Biblioteca nativa da IDE Arduino
  #include <Stepper.h>

//Definindo Constantes
  #define IN5 8
  #define IN6 9
  #define IN7 10
  #define IN8 11

 //Variáveis globais
  int SpR = 2048;
  int x, y,i,j;
  int flag = 500;
  int pos = 1;
  uint8_t velocidade = 6;
  float ang = 0;
  

  Stepper MotorPasso_el(SpR, IN5, IN6, IN7, IN8);  //Configurando um motor

void setup() 
{
  // Velocidade inicial 
  MotorPasso_el.setSpeed(velocidade);
}

void loop(){
  y = analogRead(A0);    // Eixo 1 Y1 - A0
 andarMotor(y);
}

  void andarMotor(int y){
    //ELEVACAO  
  if (y >= 800){
      for (i = pos; i <= 2048; i++) {
        MotorPasso_el.step(1);
        //Serial.print("Valor do for: ");
        //Serial.println(i);
        ang = map (i, 0, 2048, 0, 360);
        Serial.print("Angulo: ");
        Serial.println(ang);
        //Serial.print(" Analogico: ");
        //Serial.println(y);
        delay(10);
        fecharCicloMaior();
        pararMotorCima();
        }
   }else if (y <= 300){
        for (i = pos; i >= 1; i--) {
        MotorPasso_el.step(-1);
        //Serial.print("Valor do for: ");
        //Serial.println(i);
        ang = map (i, 0, 2048, 0, 360);
        Serial.print("Angulo: ");
        Serial.println(ang);
        //Serial.print(" Analogico: ");
        //Serial.println(y);
        delay(10);
        fecharCicloMenor();
        pararMotorBaixo();
        }
     }
  }

  void pararMotorCima(){
    flag = analogRead(A0);
    //Serial.print("Flag: ");
    //Serial.println(flag);  
      if(flag > 500 and flag < 550){
        pos = i;
        i = 3000;
        Serial.println("Parar motor!!!!!");
      }
  }

  void pararMotorBaixo(){
    flag = analogRead(A0);
    //Serial.print("Flag: ");
    //Serial.println(flag);  
  if(flag > 500 and flag < 550){
        pos = i;
        i = -1;
        Serial.println("Parar motor!!!!!");
      }
  }

  void fecharCicloMaior(){
    if (i == 2047){
      pos = 0;
    }
  }

  void fecharCicloMenor(){
    if (i == 1){
      pos = 2048;
    }
  }
