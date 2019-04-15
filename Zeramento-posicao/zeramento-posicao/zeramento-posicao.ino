#include <Wire.h>
#include <MechaQMC5883.h>
#include <Stepper.h>


//Definindo Constantes
  //Definindo entradas do motor de passo X:
  #define IN2 2
  #define IN3 3
  #define IN5 5
  #define IN6 6

  //Variáveis globais
    //Motor de Passo:
    int SpRX = 12800; //Definindo pulso por revolução do motor X
    uint8_t velocidade = 1; //Definição de velocidade do motor de passo
    //Sensor Magnetometro
    MechaQMC5883 qmc; //Definindo sensor magnetometro
    int qmcx, qmcy, qmcz; //Definindo variaveis para coordenadas
    int azimuth = 0; //Variavel para o Azimuth
    bool running = false;
    int calibracao = 10;
    
  //Definindo motores
  Stepper MotorPasso_X(SpRX, IN2, IN3, IN5, IN6); //Configurando um motor

void setup() {
  Wire.begin(); //Wire para o Serial Print
  Serial.begin(9600); //Configuraçao da velocidade de porta
  qmc.init(); //Inicializando Magnetometro
  //qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);
  MotorPasso_X.setSpeed(velocidade); //Configurando motor de passo com velocidade
}

void loop() {
  //Inicialização do Rastreador:
  if (running == false){
    zeramentoPosicao();
  }
  //Teste do Norte:
    else{
    qmc.read(&qmcx, &qmcy, &qmcz, &azimuth);//Print teste posicao
    Serial.print("Azimuth: ");
    Serial.println(azimuth);
  }
}

void zeramentoPosicao(){
  qmc.read(&qmcx, &qmcy, &qmcz,&azimuth); //Print teste posicao
    if (azimuth < 120 ) { //Testa se está no limite inferior
      //qmc.read(&qmcx, &qmcy, &qmcz,&azimuth); //Print teste posicao
      MotorPasso_X.step(1); //Anda sentido horario
      delay(35); //Frequencia de pulso para o motor
     }
    else if ( azimuth > 130 ) { //Testa se está no limite superior
      //qmc.read(&qmcx, &qmcy, &qmcz, &azimuth);//Print teste posicao
      MotorPasso_X.step(-1); //Anda no senti antihorario
      delay(35); //Frequencia de pulso para motor
    }
  //Serial.print("Azimuth: ");
  //Serial.println(azimuth);
  else if (azimuth == 125){
    running = true;
  }
}