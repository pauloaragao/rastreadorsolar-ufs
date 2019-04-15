#include <Arduino.h>
#include <math.h>
#include <DS1307.h>
#include <Wire.h>
#include <MechaQMC5883.h>
#include <MPU6050.h>
#include <Stepper.h>
#include <SPI.h>
#include <SD.h>

/*
Read Me:
RTC: A2, A1;

*/


//Acelerometro:
  MPU6050 mpu; //Variavel para acelerometro
// SD CARD
  const int chipSelect = 53;  //Variavel para o SDCard
//Magnetometro
  MechaQMC5883 qmc; //Variavel para Magnetometro
//RTC
  DS1307 rtc(A2, A3); //Portas Analogicas A2 e A3 RTC
  String data, hora, h, m, s, dia, mes, ano; //Variaveis para RTC

//Variaveis de Ajuste
  //Variavel de passo para o codigo
  int passoCodigo = 0;
  //Equacao de Calculo de Azimuth e Elevacao
  double LL = 33.10;                            //Longitude do Local//
  double LH = 30;                            //Longitude do Meridiano de Greenwich//
  double AO = 0;                            //Adiantamento do horário de Verão//
  //double TO;                            //Horario local//
  double fi_latitude = -10.93;                   //Latitude do Local//
  //Variavel para armazenar elevacao e azimuth atuais: 
  int elev = 0;
  int azim = 0;
  
  float Direta_ref, ajuste_elev_1, ajuste_elev_2, ajuste_azim_1, ajuste_azim_2;
  int i, j, ha, az;
  
  float    alfa_elevacao_completo;
  int    azimute_int;
  double A;
  double f;
  double delta_declinacao ;
  double delta_declinacao_graus ;
  double eq_tempo ;
  double eq_tempo_horas ;
  double H_sol ;
  double Omega_horario_angular ;
  double zenite ;
  double zenite_graus ;
  double alfa_elevacao ;
  double psi_azimute ;
  double psi_azimute_graus ;
  double Azimute_ajustado ;

  int e, correcao_e, correcao_a, a , g, posic_ant_azim;
  float hora_bb;
  int graus , azimute_anterior;
  float dda;
  int pulsos_movimentacao_azimute;
  int pulsos_movimentacao_elevacao;
  int sensor_pirel = 1; //pino referente ao pireliômetro
  int sensor_piran = 0; //pino referente ao piranômetro
  //variável usada para ler o valor do sensor em tempo real.
  int valorSensor_pirel, valorSensor_piran, Rad_difusa;

// Motor de Passo
  const int stepsPerRevolution_e = 16000;
  const int stepsPerRevolution_a = 12800;
  Stepper myStepper_a(stepsPerRevolution_e, 2, 3, 5, 6);
  Stepper myStepper_e(stepsPerRevolution_a, 7, 8, 9, 10);


void initSDCard(){
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    // don't do anything more:
    while (1);
  }
}

void initRTC(){
  rtc.halt(false);
  rtc.setDOW(SATURDAY);       //Define o dia da semana
  rtc.setTime(14, 00, 00);    //Define o horario
  rtc.setDate(02, 04, 2019);  //Define o dia, mes e ano
  //Definicoes do pino SQW/Out
  rtc.setSQWRate(SQW_RATE_1);
  rtc.enableSQW(true);
  // Para o seguidor conhecer a hora inicial
  data = rtc.getDateStr(FORMAT_SHORT);
  hora = rtc.getTimeStr();
  h = hora.substring(0, 2);
  m = hora.substring(3, 5);
  s = hora.substring(6, 8);
  dia = data.substring(0, 2);
  mes = data.substring(3, 5);
  ano = data.substring(6, 8);

  hora_bb = h.toFloat() + m.toFloat() / 60 ;
  dda = ((275*mes.toInt()/9)-30+dia.toInt()-2);
    
  if (mes < 3){ //Janeiro e Fevereiro e ano não bissexto
    dda = dda + 2;
  }else if ((ano.toInt() % 4 == 0 && (ano.toInt() % 400 == 0 || ano.toInt() % 100 != 0)) and mes > 2){//Verifica Ano Bissexto e se é maior que Fevereiro
    dda = dda + 1;  
  }
}

void initDeclinacao(){
  A = (2 * 3.141592 * (dda - 1)) / 365 ; //
  f = (2 * 3.141592 * (dda - 81)) / 364 ; //PARÂMETROS
  delta_declinacao  = 0.006918 - 0.399912 * cos(A) + 0.070257 * sin(A) - 0.006758 * cos(2 * A) + 0.000907 * sin(2 * A) - 0.002697 * cos(3 * A) + 0.00148 * sin(3 * A); //Declincao solar//
  delta_declinacao_graus = delta_declinacao * 57.2958 ;
}

void initMPU(){
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    //  delay(500);
  }
}

void initCompass(){
  qmc.init();
  qmc.setMode(Mode_Continuous, ODR_200Hz, RNG_2G, OSR_256);
}

void coordenadasCelestes(){
  eq_tempo = 9.87 * sin ( 2 * f) - 7.53 * cos (f) - 1.5 * sin (f) ; //Equacao do tempo//
  eq_tempo_horas = eq_tempo / 60 ;
  H_sol = hora_bb + eq_tempo_horas - AO + (  - LL - LH) / 15 ;  //Tempo verdadeiro do sol//
  Omega_horario_angular = ((H_sol - 12) * 15) ;         //Hora angular em//
  zenite = acos ( sin (fi_latitude * 0.0174533) * sin(delta_declinacao) + cos (fi_latitude * 0.0174533) * cos (delta_declinacao) * cos (Omega_horario_angular * 0.0174533))   ; //ângulo zenital//
  zenite_graus = zenite * 57.2958       ;
  alfa_elevacao = 90 -  zenite_graus     ;               // Elevacao//
  psi_azimute = acos ( (   ( sin(alfa_elevacao * 0.0174533) * sin (fi_latitude * 0.0174533) - sin (delta_declinacao)) /    (cos(alfa_elevacao * 0.0174533) * cos (fi_latitude * 0.0174533))   )  )  ; //Azimute//
  psi_azimute_graus = psi_azimute * 57.2958 ;

    if (Omega_horario_angular < 0) {
      Azimute_ajustado  = 180 - psi_azimute_graus ;
    }
    else if  (Omega_horario_angular > 0) {
      Azimute_ajustado  = 180 + psi_azimute_graus ;
    }
    else if  (Omega_horario_angular == 0) {
      Azimute_ajustado  = 180 ;
    }
}

void horaAtual(){
  // FORNECE A HORA SEPARADAMENTE
    data = rtc.getDateStr(FORMAT_SHORT);
    hora = rtc.getTimeStr();
    h = hora.substring(0, 2);
    m = hora.substring(3, 5);
    s = hora.substring(6, 8);
    dia = data.substring(0, 2);
    mes = data.substring(3, 5);
    ano = data.substring(6, 8);

    hora_bb = h.toFloat() + m.toFloat() / 60 ;
    Serial.print(" Hora atual em decimal: ");  Serial.print(hora_bb);
}

void posicionarAzimute(){
  if (azim + (360 - Azimute_ajustado) < abs((Azimute_ajustado - azim) ) ) {
      pulsos_movimentacao_azimute = (azim + (360 - Azimute_ajustado)  ) * 35.55 ;
      for (j = 0; j <= pulsos_movimentacao_azimute; j++) {
        myStepper_a.step(-1);
        delay(10);
      }
    }
    else   {
      pulsos_movimentacao_azimute = (Azimute_ajustado - azim) * 35.55 ;
      for (j = 0; j <= pulsos_movimentacao_azimute; j++) {
        myStepper_a.step(1);
        delay(10);
      }
    }

    azim = Azimute_ajustado;
}

void posicionarElevacao(){
  pulsos_movimentacao_elevacao = (elev - alfa_elevacao ) * 44.44 ;
    ha = pulsos_movimentacao_elevacao / (abs (pulsos_movimentacao_elevacao));
    for (i = 0; i <= (abs (pulsos_movimentacao_elevacao)); i++) {
      myStepper_e.step(ha);
      delay(10);
    }

    elev = alfa_elevacao;
}

void coletarSensores(){
  //PIRELIÔMETRO
      int valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
      valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
    //PIRANÔMETRO
      int valorSensor_piran = analogRead(sensor_piran); //ler os valores do pireliômetro
      valorSensor_piran = map (valorSensor_piran, 0, 1023, 0, 4000);
    //RADIAÇÃO DIFUSA
      delay(100);
      Rad_difusa = valorSensor_piran - valorSensor_pirel ; //Valor da radiação difusa
      delay (200); // delay para estabilizar
}

void armazenarSD(){
   File dataFile = SD.open("teste30.txt", FILE_WRITE); // coloco o nome do arquivo e abro ele
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.print("Hora"); //salvo a variável "a" por exemplo
      dataFile.print( "  "); // espaço para ter em colunas
      dataFile.print("Data"); //salvo a variável "a" por exemplo
      dataFile.print( "  "); // espaço para ter em colunas
      dataFile.print("radiação direta"); //salvo a variável "a" por exemplo
      dataFile.print( "  "); // espaço para ter em colunas
      dataFile.print("radiação Global");
      dataFile.print( "  ");
      dataFile.println("radiação difusa");

      dataFile.print(h); dataFile.print( ":"); dataFile.print(m); dataFile.print( ":"); dataFile.print(s);
      dataFile.print( "  "); // espaço para ter em colunas
      dataFile.print(dia);  dataFile.print( ":"); dataFile.print(mes); dataFile.print( ":"); dataFile.print(ano);
      dataFile.print( "  "); // espaço para ter em colunas
      dataFile.print(valorSensor_pirel); //salvo a variável "a" por exemplo
      dataFile.print( "  "); // espaço para ter em colunas
      dataFile.print(valorSensor_piran);
      dataFile.print( "  ");
      dataFile.println(Rad_difusa);
      delay(100);
      dataFile.close(); // fecha o arquivo
    }
    // if the file isn't open, pop up an error:
    else {
    }
}

void printTela(){
  Serial.print (" Azimute calculado: "); Serial.print (Azimute_ajustado); Serial.print (" Elevacao calculado: "); Serial.print (alfa_elevacao);
  Serial.print (" hora: "); Serial.print (h); Serial.print (":"); Serial.print (m); Serial.print (":"); Serial.print (s);
  Serial.print (" Rad Direta: "); Serial.print (valorSensor_pirel); Serial.print (" Rad global: "); Serial.print (valorSensor_piran); Serial.print (" Rad difusa: "); Serial.println (Rad_difusa);
  delay (600000);  // 10 minutos entre uma movimentação
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  // SD CARD
    initSDCard();
  //RTC
    initRTC();
  //Declinacao
    initDeclinacao();
  //MPU
    initMPU();
  //COMPASS
    initCompass();
  //Motores
    myStepper_e.setSpeed(1); //Motor para Elevacao setando velocidade
    myStepper_a.setSpeed(1);  //Motor para Azimute setando velocidade

}



void loop() {
  delay(2000);
  //RASTREAMENTO
  if(hora_bb <= 17) {
    /*Posicionamento Teórico*/
    //HORA ATUAL:
      horaAtual();
    //CALCULO DAS COORDENADAS CELESTES LOCAIS:
      coordenadasCelestes();
      delay (1000);
    //POSICIONAR NO AZIMUTE:
      posicionarAzimute();
      delay (2000);
    //POSICIONAR NA ELEVACAO
     posicionarElevacao();
     delay(3000);
     
    /*Coleta de Dados dos Sensores:*/
    //Coleta e Calcula sensores
    coletarSensores();
    //Armazena Valores no Cartao SD:
    armazenarSD();
    //Monitorar valores na tela
    printTela();
  }
}
