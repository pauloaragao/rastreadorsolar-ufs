/*
* Rastreador Solar  Laboratório de Conversao de Energias - Universidade Federal de Sergipe
* Discentes: José Orlando Rodrigues Santos, Paulo Vitor Aragão Silva, Henrique Silveira Alves Marques
* Orientador: Douglas Bressan Riffel
* 
* Descrição resumida do projeto
* Data: 10/10/2018
* 
* Fonte de dados, ou projeto utilizado como base
* 
* I/Os Utilizadas:
*
* Última Alteração:
* Alterações para teste: ArmazenarSD(); -> Colocar nome do arquivo , initRTC() -> Ajusta horário e dia
*/


#include <Wire.h>
#include <MechaQMC5883.h>
#include <Stepper.h>
#include <SD.h>
#include <Arduino.h>
#include <math.h>
#include <DS1307.h>
#include <MechaQMC5883.h>
#include <MPU6050.h>
#include <SPI.h>


//Definindo Constantes
  //Definindo entradas do motor de passo X:
  #define IN2 2
  #define IN3 3
  #define IN5 5
  #define IN6 6

  //Definindo entradas do motor de passo Y:
  #define IN7 7
  #define IN8 8
  #define IN9 9
  #define IN10 10

  //Variáveis globais
  //Variaveis auxiliares
      /*Parametros para calculo de declinacao*/
        double A; //Parametro para calculo da declinacao
        double f; //Parametro para calculo do tempo
        double delta_declinacao; //Declinacao em radianos
        double delta_declinacao_graus; //Declinacao em graus 
        
        /*Parametros para busca do azimute*/
        int i, j; //Variaveis auxiliares para laços
        double Azimute_ajustado ; //Angulo azimute a partir do norte
        int azim = 0;
        int pulsos_movimentacao_azimute; //variavel para pulsos de movimentacao
       
        
        /*Parametros para calculo dos corpos celestes*/
         double LL = -37.10;                            //Longitude do Local//
         double LH = 30;                            //Longitude do Meridiano de Greenwich//
         double AO = 0;                            //Adiantamento do horário de Verão//
         double fi_latitude = -10.93;                   //Latitude do Local//
         double Omega_horario_angular; //Hora angular do sol
         double eq_tempo; //Variavel que ajusta valor de hora real com a aparente (Minutos)
         double eq_tempo_horas; //Variavel transforma para horas 
         double H_sol; //Horario solar 
         double zenite; //Angulo de Zenite em radianos
         double zenite_graus;  //Angulo de Zenite em graus
         double alfa_elevacao; //Angulo de Elevacaol
         double psi_azimute; //Angulo de azimute em radianos 
         double psi_azimute_graus ; //Angulo de azimute em graus

         /*Parametros para busca da elevacao */
         int pulsos_movimentacao_elevacao; //variavel para pulsos de movimentacao
         int elev = 0;
         int ha; //Variavel para definicao do sentido da elevacao

         /*Variaveis para coleta dos sensores*/
         int valorSensor_pirel, valorSensor_piran, Rad_difusa;
         int sensor_pirel = 1; //pino referente ao pireliômetro
         int sensor_piran = 0; //pino referente ao piranômetro

         /*Variavel do cartao SD*/
         File dataFile;
         const int chipSelect = 53;  //Variavel para o SDCard

         
    //Motor de Passo:
      int SpRX = 12800; //Definindo pulso por revolução do motor X
      int SpRY = 16000;
      uint8_t velocidade = 1; //Definição de velocidade do motor de passo
      
    //Sensor Magnetometro
      MechaQMC5883 qmc; //Definindo sensor magnetometro
      int qmcx, qmcy, qmcz; //Definindo variaveis para coordenadas
      int azimuth = 0; //Variavel para o Azimuth
      int estadoCodigo = 0;
      int calibracao = 10;

    //Sensor RTC
      //RTC
        /*Function: initRTC and calcularDDA*/
        DS1307 rtc(A2, A3); //Portas Analogicas A2 e A3 RTC
        String data, hora, h, m, s, dia, mes, ano; //Variaveis para RTC
        float hora_bb; // HH:MM em somente horas (Conversao)
        float dda;// Número do dia ano


    //Acelerometro:
        MPU6050 mpu; //Variavel para acelerometro
        int roll; //Variavel global para o roll

        
  //Definindo motores
    Stepper MotorPasso_X(SpRX, IN2, IN3, IN5, IN6); //Configurando um motor
    Stepper MotorPasso_Y(SpRY, IN7, IN8, IN9, IN10); //Configurando um motor
    
void setup() {
  Wire.begin(); //Wire para o Serial Print
  Serial.begin(9600); //Configuraçao da velocidade de porta
  
  qmc.init(); //Inicializando Magnetometro
  initMPU();//Inicializando Sensor de Elevação
  initRTC(); //Inicializando o sensor RTC
  initDeclinacao();//Calculo por formula da declinacao
  initSDCard(); //Inicializacao do cartao SD
  
  MotorPasso_X.setSpeed(velocidade); //Configurando motor de passo com velocidade
  MotorPasso_Y.setSpeed(velocidade); //Configurando motor de passo com velocidade
}

void loop() {
  //Inicialização do Rastreador:
  switch (estadoCodigo) {
    case (0):
    zeramentoElevacao();
    zeramentoPosicao();
    Serial.print("Azimute: ");
    Serial.println(azimuth);
    Serial.print("Estado Logico: ");
    Serial.println(estadoCodigo);
    break;
  case (1):
    //Serial.print("Estado Logico: ");
    //Serial.println(estadoCodigo);
    qmc.read(&qmcx, &qmcy, &qmcz, &azimuth);//Print teste posicao
    Serial.print("Azimuth: ");
    Serial.println(azimuth);
    Serial.print("DDA: ");
    Serial.println(dda);
    delay(5000);
    estadoCodigo = 2;      
    break;
  default:
    horaAtual();
    coordenadasCelestes();
    posicionarAzimute();
    delay(2000);
    posicionarElevacao();
    Serial.println();
    Serial.print("Azimute Ajustado: ");
    Serial.println(Azimute_ajustado);
    Serial.print("Elevacao Ajustado: ");
    Serial.println(alfa_elevacao);
    printTela();
    armazenarSD();
    delay(360000);
    // statements
    break;
  }

}


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
  rtc.setDate(17, 04, 2019);  //Define o dia, mes e ano
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
 
  calculoDDA();
}

void calculoDDA(){
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
}

void initDeclinacao(){
  A = (2 * 3.141592 * (dda - 1)) / 365 ; //Parametro para calculo da declinacao
  f = (2 * 3.141592 * (dda - 81)) / 364 ; //Parametro para calculo do tempo
  delta_declinacao  = 0.006918 - 0.399912 * cos(A) + 0.070257 * sin(A) - 0.006758 * cos(2 * A) + 0.000907 * sin(2 * A) - 0.002697 * cos(3 * A) + 0.00148 * sin(3 * A); //Declincao solar//
  delta_declinacao_graus = delta_declinacao * 57.2958 ;
}

void initMPU(){
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    //  delay(500);
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


void posicionarAzimute(){
  if (azim + (360 - Azimute_ajustado) < abs((Azimute_ajustado - azim) ) ) {
      pulsos_movimentacao_azimute = (azim + (360 - Azimute_ajustado)  ) * 35.55;
      for (j = 0; j <= pulsos_movimentacao_azimute; j++) {
        MotorPasso_X.step(-1);
        delay(10);
      }
    }
    else   {
      pulsos_movimentacao_azimute = (Azimute_ajustado - azim) * 35.55 ;
      for (j = 0; j <= pulsos_movimentacao_azimute; j++) {
        MotorPasso_X.step(1);
        delay(10);
      }
    }

    azim = Azimute_ajustado;
}



void posicionarElevacao(){
  pulsos_movimentacao_elevacao = (elev - alfa_elevacao ) * 44.44 ; //Define o pulso da movimentacao da elevacao
  ha = pulsos_movimentacao_elevacao / (abs (pulsos_movimentacao_elevacao)); //Define o sentido da Elevacao
    for (i = 0; i <= (abs (pulsos_movimentacao_elevacao)); i++) {
      MotorPasso_Y.step(ha);
      delay(10);
    }
    elev = alfa_elevacao;
}

void coletarSensores(){
  //PIRELIÔMETRO
      valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
      valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
    //PIRANÔMETRO
      valorSensor_piran = analogRead(sensor_piran); //ler os valores do pireliômetro
      valorSensor_piran = map (valorSensor_piran, 0, 1023, 0, 4000);
    //RADIAÇÃO DIFUSA
      delay(100);
      Rad_difusa = valorSensor_piran - valorSensor_pirel ; //Valor da radiação difusa
      delay (200); // delay para estabilizar
}


void printTela(){
  Serial.println();
  Serial.print (" Azimute calculado: "); Serial.println (Azimute_ajustado); Serial.print (" Elevacao calculado: "); Serial.println (alfa_elevacao);
  Serial.print (" hora: "); Serial.print (h); Serial.print (":"); Serial.print (m); Serial.print (":"); Serial.println (s);
  Serial.print (" Rad Direta: "); Serial.println (valorSensor_pirel); Serial.print (" Rad global: "); Serial.println (valorSensor_piran); 
  Serial.print (" Rad difusa: "); Serial.println (Rad_difusa);
}


void zeramentoElevacao(){
  // Read normalized values
  Vector normAccel = mpu.readNormalizeAccel();
  roll = (atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI;
  Serial.print("Roll: ");
  Serial.println(roll);

    if (roll < 1 ) { //Testa se está no limite inferior
      //qmc.read(&qmcx, &qmcy, &qmcz,&azimuth); //Print teste posicao
      MotorPasso_Y.step(-1); //Anda sentido horario
      delay(10); //Frequencia de pulso para o motor
     }
    else if (roll > 1) { //Testa se está no limite superior
      //qmc.read(&qmcx, &qmcy, &qmcz, &azimuth);//Print teste posicao
      MotorPasso_Y.step(1); //Anda no senti antihorario
      delay(10); //Frequencia de pulso para motor
    }
    else if ((roll == 1)and(azimuth == 115)){
      estadoCodigo = 1;
    }
}

void zeramentoPosicao(){
  qmc.read(&qmcx, &qmcy, &qmcz,&azimuth); //Print teste posicao
     if (azimuth < 115 ) { //Testa se está no limite inferior
      //qmc.read(&qmcx, &qmcy, &qmcz,&azimuth); //Print teste posicao
      MotorPasso_X.step(1); //Anda sentido horario
      delay(10); //Frequencia de pulso para o motor
     }
    else if ( azimuth > 117) { //Testa se está no limite superior
      //qmc.read(&qmcx, &qmcy, &qmcz, &azimuth);//Print teste posicao
      MotorPasso_X.step(-1); //Anda no senti antihorario
      delay(10); //Frequencia de pulso para motor
    }
  //Serial.print("Azimuth: ");
  //Serial.println(azimuth);
  else if ((azimuth == 115)and(roll==1)){
    estadoCodigo = 1;
  }
}


void armazenarSD(){
   dataFile = SD.open("teste30.txt", FILE_WRITE); // coloco o nome do arquivo e abro ele
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
