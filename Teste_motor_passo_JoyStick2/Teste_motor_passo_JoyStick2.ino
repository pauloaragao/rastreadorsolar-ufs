// Biblioteca nativa da IDE Arduino
  #include <Stepper.h>

//Definindo Constantes
  //Definindo entradas do motor de passo X:
  #define IN4 4
  #define IN5 5
  #define IN6 6
  #define IN7 7
  //Definindo entradas do motor de passo Y:
  #define IN8 8
  #define IN9 9
  #define IN10 10
  #define IN11 11
  
 //Variáveis globais
  int SpRY = 16000; //Definindo pulso por revolução do motor Y
  int SpRX = 12800; //Definindo pulso por revolução do motor X
  int x, y; //Variavel para leitura analogica dos motores X e Y 
  int posx = 0; //Variável que armazena a posição de X
  int posy = 0; //Variável que armazena a posição de Y
  uint8_t velocidade = 1; //Definição de velocidade do motor de passo
  float ang = 0; //Variavel que armazena o valor do angulo
  
  //Definindo motores
  Stepper MotorPasso_Y(SpRY, IN8, IN9, IN10, IN11);  //Configurando um motor
  Stepper MotorPasso_X(SpRX, IN4, IN5, IN6, IN7);  //Configurando um motor

void setup() 
{
  //Velocidade inicial 
  MotorPasso_Y.setSpeed(velocidade); //Configurando motor de passo com velocidade
  MotorPasso_X.setSpeed(velocidade); //Configurando motor de passo com velocidade
}

void loop(){
  y = analogRead(A0); //Ler o valor do Analógico Y
  x = analogRead(A1); //Ler o valor do Analógico X
  andarMotorY(y); //Anda motor em Y
  andarMotorX(x); //Anda motor em X
}

void andarMotorY (int y){ 
  if (y >= 800){ //Semi-ciclo positivo
        MotorPasso_Y.step(1); //Passo do motor
        //Serial.print("Valor do for: ");
        //Serial.println(i);
        //ang = map (pxosy, 0, SpRY, 0, 360);
        ang = posy*(360.00/SpRY); //Calculo em float para angulo
        Serial.print("Angulo PosY: ");
        Serial.println(ang);
        //Serial.println(posy);
        //Serial.print(" Posicao: ");
        //Serial.println(pos);
        delay(10); //Delay para o motor
        posy = posy + 1; //Adiciona uma posição do motor
        if (posy == SpRY+1){
          posy = 0; //Fecha o cilo maior quando chegar em 360 graus
        }
   }else if (y <= 300){
        MotorPasso_Y.step(-1);//Passo do motor
        //Serial.print("Valor do for: ");
        //Serial.println(i);
        //ang = map (posy, 0, SpRY, 0, 360);
        ang = posy*(360.00/SpRY); //Calculo em float para angulo
        Serial.print("Angulo PosY: ");
        Serial.println(ang);
        //Serial.print(" Analogico: ");
        //Serial.println(y);
        delay(10); //Delay para o motor
        posy = posy - 1;//Decrementa uma posição do motor
        if (posy == -1){
          posy = SpRY; //Fecha o ciclo menor quando chegar em 0 graus
        }
        }
     }

void andarMotorX (int x){ 
  if (x >= 800){ //Semi-ciclo positivo
        MotorPasso_X.step(1); //Passo do motor
        //Serial.print("Valor do for: ");
        //Serial.println(i);
        //ang = map (posx, 0, SpRX, 0, 360);
        ang = posx*(360.00/SpRX);//Calculo em float para angulo
        Serial.print("Angulo PosX: ");
        Serial.println(ang);
        //Serial.print(" Posicao: ");
        Serial.println(x);
        delay(10);//Delay para o motor
        posx = posx + 1;//Adiciona uma posição do motor
        if (posx == SpRX+1){
          posx = 0;//Fecha o cilo maior quando chegar em 360 graus
        }
   }else if (x <= 300){//Semi-ciclo negativo
        MotorPasso_X.step(-1);//Passo do motor
        //Serial.print("Valor do for: ");
        //Serial.println(i);
        //ang = map (posx, 0, SpRX, 0, 360);
        ang = posx*(360.00/SpRX);
        Serial.print("Angulo PosX: ");
        Serial.println(ang);
        //Serial.print(" Analogico: ");
        Serial.println(x);
        delay(10);//Delay para o motor
        posx = posx - 1;//Decrementa uma posição do motor
        if (posx == -1){
          posx = SpRX; //Fecha o ciclo menor quando chegar em 0 graus
        }
        }
     }


 
