#include <Arduino.h>
#include <math.h>
#include <DS1307.h>
#include <Wire.h>
#include <MechaQMC5883.h>
#include <MPU6050.h>
#include <Stepper.h>
#include <SPI.h>
#include <SD.h>

//MPU
MPU6050 mpu;

// SD CARD
const int chipSelect = 53; // ATENÇÃO TENHO QUE ALTERAR A ENTRADA DOS STEPPER!!!!!

//COMPASS
MechaQMC5883 qmc;

//RTC
DS1307 rtc(A2, A3);
String data, hora, h, m, s, dia, mes, ano;
float Direta_ref, ajuste_elev_1, ajuste_elev_2, ajuste_azim_1, ajuste_azim_2;
//DECLARAÇÃO DE VARIÁVEIS//
int i, j, ha, az;
double LL;                            //Longitude do Local//
double LH;                            //Longitude do Meridiano de Greenwich//
double AO;                            //Adiantamento do horário de Verão//
double TO;                            //Horario local//
double fi_latitude;                   //Latitude do Local//
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
int     elev, azim;
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

// MOTORES
const int stepsPerRevolution_e = 16000;
const int stepsPerRevolution_a = 12800;
Stepper myStepper_a(stepsPerRevolution_e, 2, 3, 5, 6);
Stepper myStepper_e(stepsPerRevolution_a, 7, 8, 9, 10);

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // SD CARD
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    // don't do anything more:
    while (1);
  }

  //RTC
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

  if (ano.toInt() % 4 == 0 && (ano.toInt() % 400 == 0 || ano.toInt() % 100 != 0)) {
    switch (mes.toInt()) {
      case 1:
        dda = dia.toFloat();
        break;
      case 2:
        dda = dia.toFloat() + 31;
        break;
      case 3:
        dda = dia.toFloat() + 60;
        break;
      case 4:
        dda = dia.toFloat() + 91;
        break;
      case 5:
        dda = dia.toFloat() + 121;
        break;
      case 6:
        dda = dia.toFloat() + 152;
        break;
      case 7:
        dda = dia.toFloat() + 182;
        break;
      case 8:
        dda = dia.toFloat() + 213;
        break;
      case 9:
        dda = dia.toFloat() + 244;
        break;
      case 10:
        dda = dia.toFloat() + 274;
        break;
      case 11:
        dda = dia.toFloat() + 305;
        break;
      case 12:
        dda = dia.toFloat() + 335;
        break;
      default:
        break;
    }
  }

  else {
    switch (mes.toInt()) {
      case 1:
        dda = dia.toFloat();
        break;
      case 2:
        dda = dia.toFloat() + 31;
        break;
      case 3:
        dda = dia.toFloat() + 59;
        break;
      case 4:
        dda = dia.toFloat() + 90;
        break;
      case 5:
        dda = dia.toFloat() + 120;
        break;
      case 6:
        dda = dia.toFloat() + 151;
        break;
      case 7:
        dda = dia.toFloat() + 181;
        break;
      case 8:
        dda = dia.toFloat() + 212;
        break;
      case 9:
        dda = dia.toFloat() + 243;
        break;
      case 10:
        dda = dia.toFloat() + 273;
        break;
      case 11:
        dda = dia.toFloat() + 304;
        break;
      case 12:
        dda = dia.toFloat() + 334;
        break;
      default:
        break;
    }
  }

  LL = - 37.10  ;                    //Longitude do Local//
  LH = 30    ;                       //Longitude do Meridiano de Greenwich//
  AO = 0    ;                        //Adiantamento do horário de Verão//
  //   TO = 1     ;                       //Horario local inicial//
  fi_latitude = - 10.93  ;              //Latitude do Local//
  A = (2 * 3.141592 * (dda - 1)) / 365 ; //PARÂMETROS
  f = (2 * 3.141592 * (dda - 81)) / 364 ; //PARÂMETROS
  delta_declinacao  = 0.006918 - 0.399912 * cos(A) + 0.070257 * sin(A) - 0.006758 * cos(2 * A) + 0.000907 * sin(2 * A) - 0.002697 * cos(3 * A) + 0.00148 * sin(3 * A); //Declincao solar//
  delta_declinacao_graus = delta_declinacao * 57.2958 ;
  eq_tempo = 9.87 * sin ( 2 * f) - 7.53 * cos (f) - 1.5 * sin (f) ; //Equacao do tempo//
  eq_tempo_horas = eq_tempo / 60 ;

  //MPU
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    //  delay(500);
  }

  //COMPASS
  qmc.init();
  qmc.setMode(Mode_Continuous, ODR_200Hz, RNG_2G, OSR_256);

  e = 1; //contador para autoposicionado elevacao
  myStepper_e.setSpeed(1);
  a = 1; //contador para autoposicionado azimute
  myStepper_a.setSpeed(1);
  g = 1; //contador para mensagem de posicioando executado
  elev = 0; // Loop do posicionamento da Elevação
  azim = 0;  // Loop do posicionamento da Azimute
  posic_ant_azim = 0;


  // Read normalized values
  Vector normAccel = mpu.readNormalizeAccel();
  // Calculate Pitch & Roll
  int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis * normAccel.YAxis + normAccel.ZAxis * normAccel.ZAxis)) * 180.0) / M_PI;
  int roll = (atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI;
  if ((roll - 2) <= 0) {
    correcao_e = (roll - 2) * 44.44 ;
    for (i = 0; i <= correcao_e; i++) {

      myStepper_e.step(1);
      delay (10);

    }

  }

  //ZERAMENTO NA ELEVAÇÃO
  if ((roll - 2) > 0) {
    correcao_e = ((roll - 2)) * 44.44 ;
    for (i = 0; i <= correcao_e; i++) {

      myStepper_e.step(1);
      delay (10);

    }
  }

  //ZERAMENTO NO AZIMUTE
  int x, y, z;
  int azimuth;
  //float azimuth; //is supporting float too
  qmc.read(&x, &y, &z, &azimuth);
  azimuth = qmc.azimuth(&y, &x); //you can get custom azimuth
  if ( azimuth >= 0 && azimuth < 135) {
    graus = 0.46666666 * azimuth + 297;
    Serial.print ("   azimute zerado: ");      Serial.println (graus);
  }
  if ( azimuth >= 135 && azimuth <= 206) {
    graus =  1.26760563380282  * azimuth - 171.12676056338;
    Serial.print ("   azimute zerado: ");      Serial.println (graus);
  }
  if ( azimuth > 206 && azimuth <= 253) {
    graus =  1.9148936170212 * azimuth - 304.468085106383;
    Serial.print ("   azimute zerado: ");      Serial.println (graus);
  }
  if ( azimuth > 253 && azimuth <= 311) {
    graus =  1.55172413793103 * azimuth - 212.58620689655;
    Serial.print ("   azimute zerado: ");      Serial.println (graus);
  }
  if ( azimuth > 311 && azimuth < 360) {
    graus = 0.55102040 * azimuth + 98.63265306;
    Serial.print ("   azimute zerado: ");      Serial.println (graus);
  }


  if (graus >= 0 && graus < 180) {
    correcao_a = graus * 35.55;
    for (ha = 0; ha <= correcao_a; ha++) {
      myStepper_a.step(-1);
      delay (10);

    }

  }
  if (graus >= 180 && graus <= 360) {
    correcao_a = (360 - graus) * 35.55;
    for (ha = 0; ha <= correcao_a; ha++) {
      myStepper_a.step(1);
      delay (10);

    }
  }
}

void loop() {

  delay(2000);

  //RASTREAMENTO

  while (hora_bb <= 17) {

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

    //CALCULO DAS COORDENADAS CELESTES LOCAIS
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

    delay (1000);

    //POSICIONAR NO AZIMUTE
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

    delay (2000);

    //POSICIONAR NA ELEVACAO
    pulsos_movimentacao_elevacao = (elev - alfa_elevacao ) * 44.44 ;
    ha = pulsos_movimentacao_elevacao / (abs (pulsos_movimentacao_elevacao));
    for (i = 0; i <= (abs (pulsos_movimentacao_elevacao)); i++) {
      myStepper_e.step(ha);
      delay(10);
    }

    elev = alfa_elevacao;

    delay(3000);
    //LER OS VALORES DOS SENSORES NA POSIÇÃO TEÓRICA
    valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
    valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
    Direta_ref = valorSensor_pirel; //ARMAZENA O VALOR DO PIRELIÔMETRO NA POSIÇÃO TÉORICA
    delay(200);

       //REFINAMENTO DO RASTREIO//

    //REFINAMENT0 ELEVACAO
    for (i = 0; i == 45; i++) {
      myStepper_e.step(-1); // 1 GRAU PARA CIMA
      delay(10);
    }
    valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
    valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
    ajuste_elev_1 = valorSensor_pirel;
delay(100);
    if (Direta_ref < ajuste_elev_1) {
      for (i = 0; i == 45; i++) {
        myStepper_e.step(-1); // + 1 GRAU PARA CIMA
        delay(10);
      }
      valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
      valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
      ajuste_elev_2 = valorSensor_pirel;
delay(100);
      if (ajuste_elev_2 > ajuste_elev_1) {
        valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
        valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
      }
      else {
        for (i = 0; i == 45; i++) {
          myStepper_e.step(1); // 1 GRAU PARA BAIXO
          delay(10);
        }
        valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
        valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
delay(100);
      }
    }

    //OUTRO LADO
    else {
      for (i = 0; i == 89; i++) {
        myStepper_e.step(1); // + 2 GRAU PARA BAIXO
        delay(40);
      }
      valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
      valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
      ajuste_elev_1 = valorSensor_pirel;
delay(100);
      if (ajuste_elev_1 > Direta_ref) {
        for (i = 0; i == 45; i++) {
          myStepper_e.step(1); // +1  GRAU PARA BAIXO
          delay(40);
        }

        valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
        valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
        ajuste_elev_2 = valorSensor_pirel;
      }
delay(100);
      if (ajuste_elev_2 > ajuste_elev_1) {
        valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
        valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
      }
      else {
        for (i = 0; i == 45; i++) {
          myStepper_e.step(-1); // + 1 GRAU PARA CIMA
          delay(40);
        }
        valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
        valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
      }
    }
delay(200);

    //AZIMUTE
    //LER OS VALORES DOS SENSORES NA POSIÇÃO TEÓRICA
    valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
    valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
    Direta_ref = valorSensor_pirel; //ARMAZENA O VALOR DO PIRELIÔMETRO NA POSIÇÃO TÉORICA
    delay(200);

    for (j = 0; j == 36; j++) {
      myStepper_a.step(1); // 1 GRAU PARA DIREITA
      delay(40);
    }
    valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
    valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
    ajuste_azim_1 = valorSensor_pirel;

    if (Direta_ref < ajuste_azim_1) {
      for (j = 0; j == 36; j++) {
        myStepper_a.step(1); // + 1 GRAU PARA DIREITA
        delay(40);
      }
      valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
      valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
      ajuste_azim_2 = valorSensor_pirel;

      if (ajuste_azim_2 > ajuste_azim_1) {
        valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
        valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
      }
      else {
        for (j = 0; j == 36; j++) {
          myStepper_a.step(-1); // 1 GRAU PARA ESQUERDA
          delay(40);
        }
        valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
        valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);

      }
    }

    //OUTRO LADO

    else {
      for (j = 0; j == 71; j++) {
        myStepper_a.step(-1); // + 2 GRAU PARA ESQUERDA
        delay(40);
      }
      valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
      valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
      ajuste_azim_1 = valorSensor_pirel;

      if (ajuste_azim_1 > Direta_ref) {
        for (j = 0; j == 36; j++) {
          myStepper_a.step(-1); // + 1  GRAU PARA ESQUERDA
          delay(40);
        }

        valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
        valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
        ajuste_azim_2 = valorSensor_pirel;
      }
      if (ajuste_azim_2 > ajuste_azim_1) {
        valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
        valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
      }
      else {
        for (j = 0; j == 45; j++) {
          myStepper_a.step(1); //  1 GRAU PARA DIREITA
          delay(40);
        }
        valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
        valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
      }
    }

    delay(200);

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
    // SD CARD
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

    Serial.print (" Azimute calculado: "); Serial.print (Azimute_ajustado); Serial.print (" Elevacao calculado: "); Serial.print (alfa_elevacao);
    Serial.print (" hora: "); Serial.print (h); Serial.print (":"); Serial.print (m); Serial.print (":"); Serial.print (s);
    Serial.print (" Rad Direta: "); Serial.print (valorSensor_pirel); Serial.print (" Rad global: "); Serial.print (valorSensor_piran); Serial.print (" Rad difusa: "); Serial.println (Rad_difusa);
    delay (600000);  // 10 minutos entre uma movimentação

  }
}
