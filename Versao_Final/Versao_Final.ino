/*
  Rastreador Solar - Laboratório de Conversao de Energia - Universidade Federal de Sergipe
  Discentes: José Orlando Rodrigues Santos, Paulo Vitor Aragão Silva, Henrique Silveira Alves Marques
  Orientador: Douglas Bressan Riffel
  Descrição resumida do projeto
  Data: 10/10/2018
  Fonte de dados, ou projeto utilizado como base
  I/Os Utilizadas:
  Última Alteração: 18/04/2019
  Alterações para teste: ArmazenarSD(); -> Colocar nome do arquivo , initRTC() -> Ajusta horário e dia
*/

//=========================== Bibliotecas =========================//
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

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <PS2X_lib.h>

#include <Gaussian.h>
#include <LinkedList.h>
#include <GaussianAverage.h>

//=========================== Parametros do SdCard =========================//
String Nome_Arq = "Arquivo1.csv";
File dataFile;//Variavel para armazenamento do arquivo
const int chipSelect = 53;  //Variavel para o SDCard
int cont; //cont é para armazenar passos de coleta


//=========================== Parametros medias =========================//
GaussianAverage myMovingAverage_1(10);


//=========================== Variaveis dos motores =========================//
//Definindo entradas do motor de passo X:
#define IN2 22 // elevacao
#define IN3 23 // elevacao
#define IN5 24 // elevacao
#define IN6 25 // elevacao

//Definindo entradas do motor de passo Y:
#define IN7 28 //azimute
#define IN8 29 //azimute
#define IN9 30 //azimute
#define IN10 31 //azimute

//================ Configuração da Tela TFT =====================//
//Definicao de cores
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

//CS, CD, WR, RD, RESET
Adafruit_TFTLCD tft(A3, A2, A1, A0, A4);

//=========================== Variaveis do controle =========================//
PS2X ps2x;
int stepCount = 0;  // número de passos que o motor tenha tomado
int error = 0;
int type = 0;
byte vibrate = 0;
//BLOCO PS2


//=========================== Variaveis globais =========================//
/*Parametros para calculo de declinacao*/
double A; //Parametro para calculo da declinacao
double f; //Parametro para calculo do tempo
double delta_declinacao; //Declinacao em radianos
double delta_declinacao_graus; //Declinacao em graus

/*Parametros para refinamento*/
int filtred_pirel;
int tolerancia = 5;
int ajuste_azim_1;
int Direta_ref ;
int zer, k;
  //int contx;
  //int conty;

/*Parametros para busca do azimute*/
int i, j; //Variaveis auxiliares para laços
double Azimute_ajustado ; //Angulo azimute a partir do norte
double azim = 0; //Passado
int pulsos_movimentacao_azimute; //variavel para pulsos de movimentacao
int ff;

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
double elev = 0;
int ha; //Variavel para definicao do sentido da elevacao
int hb; //Variavel para definicao do sentido do azimute 

/*Variaveis para coleta dos sensores*/
int valorSensor_pirel, valorSensor_piran, Rad_difusa;
int sensor_pirel = 8; //pino referente ao pireliômetro
int sensor_piran = 9; //pino referente ao piranômetro
int global_real; //Variavel para a coleta da irradiacao global


/*Variaveis para o refinamento*/
int refinamento[3];
int caranguejo[2];

//Motor de Passo:
int SpRX = 12800; //Definindo pulso por revolução do motor X
int SpRY = 16000;
uint8_t velocidade = 1; //Definição de velocidade do motor de passo

//Sensor Magnetometro
MechaQMC5883 qmc; //Definindo sensor magnetometro
int qmcx, qmcy, qmcz; //Definindo variaveis para coordenadas
int azimuth = 0; //Variavel para o Azimuth
int estadoCodigo;


//Sensor RTC
//RTC
/*Function: initRTC and calcularDDA*/
DS1307 rtc(A10 , A11); //Portas Analogicas A2 e A3 RTC
String data, hora, h, m, s, dia, mes, ano; //Variaveis para RTC
float hora_bb; // HH:MM em somente horas (Conversao)
float dda;// Número do dia ano


//Acelerometro:
MPU6050 mpu; //Variavel para acelerometro
int roll; //Variavel global para o roll

//Definindo motores
Stepper MotorPasso_Y(SpRY, IN2, IN3, IN5, IN6); //Configurando um motor
Stepper MotorPasso_X(SpRX, IN7, IN8, IN9, IN10); //Configurando um motor

void setup() {
  Wire.begin(); //Wire para o Serial Print
  
  Serial.begin(9600); //Configuraçao da velocidade de porta
  Serial.println("passando no setup");
 delay(1000);
 //=========================== Incialização Tela TFT ===================================//

  tft.reset();
  delay(100);
  //tft.begin(0x9341) //Use esta linha para o controlador 9341
  tft.begin(0x9325);
  Serial.println("Inicializando Tela");
  teste_retangulo_cheio(YELLOW, BLUE);
  //teste_triangulos(GREEN);
  //teste_circulos(MAGENTA, BLACK); 
  delay(100);

  apresentacao();
  Serial.println("Apresentação");
  delay(500);
//======================== Inicialização dos Sensores =====================//
  tft.setRotation(1);
  tft.fillScreen(WHITE);
  tft.setTextColor(BLACK);
  tft.setTextSize(1);
  initRTC(); //Inicializando o sensor RTC
  Serial.println("rtc ok");
  delay(1000);
  qmc.init(); //Inicializando Magnetometro
   Serial.println("qmc ok");
   tft.setTextSize(1);
   tft.setCursor(15, 10);
   tft.print("qmc ok");
   delay(1000);
  initMPU();//Inicializando Sensor de Elevação
   Serial.println("mpu ok");
   tft.setCursor(15, 20);
   tft.print("mpu ok");
    delay(1000);
  initDeclinacao();//Calculo por formula da declinacao
  Serial.println("Decl ok");
  tft.setCursor(15, 30);
   tft.print("Declin ok");
  delay(1000);

  
  initSDCard(); //Inicializacao do cartao SD
  Serial.println("SD ok");
  delay(1000);

//======================== Configuração do Controle =====================//
 error = ps2x.config_gamepad(13,11,10,12, true, true);
 ps2x.enableRumble(); //Ativar vibração do controle
 ps2x.enablePressures(); //Ativar modo analógico do controle 

//======================== Configuração de Velocidade dos Motores =====================//
  MotorPasso_X.setSpeed(velocidade); //Configurando motor de passo com velocidade
  MotorPasso_Y.setSpeed(velocidade); //Configurando motor de passo com velocidade
   
//=========================== Incialização da Tela  ===================================//
  tft.setRotation(1);
  tft.fillScreen(BLACK);
  
  printStaticTela();
  
  ajuste_azim_1 = 0;
  delay(2000);
  
  estadoCodigo = 0;
  
  Serial.println("estado codigo 0");
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(2);
  tft.setCursor(280, 215);
  tft.println(estadoCodigo);
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(1);
  tft.setCursor(80, 205);
  tft.println("Iniciando Zeramento Elev. ");
  tft.setTextColor(BLACK, GREEN);
  tft.setTextSize(2);
  tft.setCursor(115, 35);
  tft.println("           ");
  tft.setCursor(115, 35);
  tft.println(" AUTOMATICO"); 
//tft.println(" Zeramento de elevacao OK ");
  

}

void loop() {
  //Inicialização do Rastreador:
  switch (estadoCodigo) {
    case (0):
      Texto_Data();
      zeramentoElevacao();
      tft.setTextColor(BLACK, GREEN);
      tft.setTextSize(2);
      tft.setCursor(115, 35);
      tft.println("           ");
      tft.setCursor(115, 35);
      tft.println(" AUTOMATICO");
   //   Serial.print("Estado Logico: ");
   //   Serial.println(estadoCodigo);
   //   Serial.print("Elevacao: ");
   //   Serial.println(azimuth);
      break;
    case (1):
      zeramentoPosicao();

        tft.setTextColor(BLACK, GREEN);
        tft.setTextSize(2);
        tft.setCursor(115, 35);
        tft.println("           ");
        tft.setCursor(115, 35);
        tft.println(" AUTOMATICO");
   //   Serial.println("___________________________");
   //   Serial.print("Estado Logico: ");
   //   Serial.println(estadoCodigo);
    //  Serial.print("Azimute: ");
   //   Serial.println(azimuth);
      break;

    case (2):
      Serial.println("___________________________");
      Serial.print("Estado Logico: ");
      Serial.println(estadoCodigo);
        tft.setTextColor(BLACK, GREEN);
        tft.setTextSize(2);
        tft.setCursor(115, 35);
        tft.println("           ");
        tft.setCursor(115, 35);
        tft.println(" AUTOMATICO");
      //qmc.read(&qmcx, &qmcy, &qmcz, &azimuth);//Print teste posicao
      delay(5000);
      Texto_Data();
      horaAtual();
      coordenadasCelestes();
      posicionarAzimute();
      delay(2000);
      posicionarElevacao();
      cont = 0;
      estadoCodigo = 3;
      /*
      Serial.print("Azimuth: ");
      Serial.println(azimuth);
      Serial.print("DDA: ");
      Serial.println(dda);
      Serial.print("Azimute Ajustado: ");
      Serial.println(Azimute_ajustado);
      Serial.print("Elevacao Ajustado: ");
      Serial.println(alfa_elevacao);
      */
      break;
    case(3):
      tft.setTextColor(BLACK, GREEN);
      tft.setTextSize(2);
      tft.setCursor(115, 35);
      tft.println("           ");
      tft.setCursor(115, 35);
      tft.println(" AUTOMATICO");
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(2);
    tft.setCursor(280, 215);
    tft.println(estadoCodigo);

    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(1);
    tft.setCursor(80, 205);
    tft.println("Verificacao de Refinamento");
    tft.setCursor(85, 220);
    tft.println("      ");
    
      Serial.println  ("___________________________");
      Serial.print("Estado Logico: ");
      Serial.println(estadoCodigo);
      Texto_Data();
      decisao_refinamento();
      
     break;
     case(4):
      //Serial.println  ("___________________________");
      //Serial.print("Estado Logico: ");
      //Serial.println(estadoCodigo);
        tft.setTextColor(BLACK, GREEN);
        tft.setTextSize(2);
        tft.setCursor(115, 35);
        tft.println("           ");
        tft.setCursor(115, 35);
        tft.println(" MANUAL    ");
        tft.setTextColor(WHITE, BLACK);
      tft.setTextSize(1);
      tft.setCursor(80, 205);
      tft.println(" Modo Manual (Controle)   ");
      tft.setCursor(80, 220);
      tft.println("                          ");
   Texto_Data();
   controle_manual();
   
      
      break;
     case(5):
     Serial.println  ("___________________________");
      Serial.print("Estado Logico: ");
      Serial.println(estadoCodigo);
        tft.setTextColor(BLACK, GREEN);
        tft.setTextSize(2);
        tft.setCursor(115, 35);
        tft.println("           ");
        tft.setCursor(115, 35);
        tft.println(" AUTOMATICO");
      refinamento_automatico();
      break;
     
     case(6):

      Serial.println  ("___________________________");
      Serial.print("Estado Logico: ");
      Serial.println(estadoCodigo);
        tft.setTextColor(BLACK, GREEN);
        tft.setTextSize(2);
        tft.setCursor(115, 35);
        tft.println("           ");
        tft.setCursor(115, 35);
        tft.println(" AUTOMATICO");
      Texto_Data();
      refinamento_automatico_elev();
     
      break;
      
    default:
      Serial.println();
      Serial.println();
        tft.setTextColor(BLACK, GREEN);
        tft.setTextSize(2);
        tft.setCursor(115, 35);
        tft.println("           ");
        tft.setCursor(115, 35);
        tft.println(" AUTOMATICO");
      controle_manual();
      Texto_Data();
      coletarSensores();
      printTela();
      armazenarSD();
      cont++;
      Serial.print("Contador do Estágio de Coleta: ");
      Serial.println(cont);
      tft.setTextColor(WHITE, BLACK);
      tft.setTextSize(1);
      tft.setCursor(80, 205);
      tft.println("                          ");
      tft.setCursor(80, 205);
      tft.println("Coletando Dados Irradiacao");
      tft.setCursor(80, 220);
      tft.println("                          ");
      tft.setCursor(85, 220);
      tft.print(cont);tft.println(" /120 seg");
      tft.setTextColor(WHITE, BLACK);
      tft.setTextSize(2);
      tft.setCursor(280, 215);
      tft.println("  ");
      tft.setCursor(280, 215);
      tft.println(estadoCodigo);
      
      //if (cont == 240) 
      if (cont == 120){
        estadoCodigo = 2;
        Serial.println("_______________________________________________________________________________________");
      }
      delay(1000);
      // statements
      //break;
  }

}


void initSDCard() {
 tft.setRotation(1);
  tft.setTextSize(2);
  tft.setCursor(5, 120);
  tft.println("Inicializando SD card.....");
  Serial.print("Inicializando SD card.....");     //Escreve frase de inicialização
  delay(100);
  if (!SD.begin(chipSelect))                              //Testa se o cartão inicializa com sucesso
  {
    Serial.println("Falha no cartao!");           //Não, falha no cartão
    tft.setCursor(5, 145);
    tft.println("Falha no cartao!");
    return;                                       //retorna
  }
  delay(100);
  Serial.println("Sucesso na inicializacao!");    //Sim, inicialização ok
  tft.setCursor(5, 145);
  tft.println("Sucesso na inicializacao!");
  
  delay(100);
  dataFile = SD.open(Nome_Arq, FILE_WRITE);     //Abre arquivo para escrita
  
  
  if (dataFile)                                     //Arquivo aberto com sucesso?
  {                                               //Sim...
    String header = "dia, mes, ano, h, m, s, G_Dir, GHI, G_Dif"; 
    dataFile.println(header);
    dataFile.close();

  } //end if
  else                                            //Não...
  {

    Serial.println("Erro ao abrir arquivo");      //Informa que há algum erro
    tft.setCursor(5, 170);
    tft.println("Erro ao abrir arquivo");
  }
}

void initRTC() {
  
  rtc.halt(false);
 // rtc.setDOW(TUESDAY);       //{"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"}
 // rtc.setTime(10, 23, 00);    //Define o horario
 // rtc.setDate(21, 05, 2019);  //Define o dia, mes e ano
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
  Serial.print("DATA: ");
  if (dia < 10) {Serial.print("0");}Serial.print(dia);Serial.print("/");
  if (mes < 10) {Serial.print("0"); }Serial.print(mes);   Serial.print("/");
  Serial.println(ano);
  Serial.print("HORA: "); 
  if (h < 10) { Serial.print("0"); }Serial.print(h);Serial.print(":");
  if (m < 10) { Serial.print("0");   }Serial.print(m);Serial.print(":");
  if (s < 10){ Serial.print("0");}Serial.println(s);

  tft.setRotation(1);
  //tft.fillScreen(WHITE);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(15, 60);
  tft.print("DATA: ");
  if (dia < 10) {tft.print("0");}tft.print(dia);tft.print("/");
  if (mes < 10) {tft.print("0"); }tft.print(mes);   tft.print("/");
  tft.println(ano);
  tft.setCursor(15, 90);
  tft.print("HORA: "); 
  if (h < 10) { tft.print("0"); }tft.print(h);tft.print(":");
  if (m < 10) { tft.print("0");   }tft.print(m);tft.print(":");
  if (s < 10){ tft.print("0");}tft.print(s);

  hora_bb = h.toFloat() + m.toFloat() / 60 ;

  calculoDDA();
}

void calculoDDA() {
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

void initDeclinacao() {
  A = (2 * 3.141592 * (dda - 1)) / 365 ; //Parametro para calculo da declinacao
  f = (2 * 3.141592 * (dda - 81)) / 364 ; //Parametro para calculo do tempo
  delta_declinacao  = 0.006918 - 0.399912 * cos(A) + 0.070257 * sin(A) - 0.006758 * cos(2 * A) + 0.000907 * sin(2 * A) - 0.002697 * cos(3 * A) + 0.00148 * sin(3 * A); //Declincao solar//
  delta_declinacao_graus = delta_declinacao * 57.2958 ;
}

void initMPU() {
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    //  delay(500);
  }
}


void horaAtual() {
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
  //Serial.print(" Hora atual em decimal: ");  Serial.println(hora_bb);
}

void coordenadasCelestes() {
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


void posicionarAzimute() {
  Serial.println("Azimute Anterior: ");
  Serial.println(azim);
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(1);
  tft.setCursor(40, 210);
  tft.println("     ");
  tft.setCursor(40, 210);
  tft.println(azim,1);
  Serial.println("Azimute Ajustado: ");
  Serial.println(Azimute_ajustado);
  tft.setTextColor(WHITE, BLUE);
  tft.setTextSize(2);
  tft.setCursor(150, 57);
  tft.println("     ");
  tft.setCursor(150, 57);
  tft.println(Azimute_ajustado,1);
  
  if (azim + (360 - Azimute_ajustado) < (abs((Azimute_ajustado - azim) ) )) {
    pulsos_movimentacao_azimute = (abs((azim + (360 - Azimute_ajustado) )) * 35);
      tft.setTextColor(WHITE, BLACK);
      tft.setTextSize(1);
      tft.setCursor(80, 205);
      tft.println(" Pulsos Moviv. Azimute    ");
      tft.setCursor(80, 220);
      tft.println("                          ");
      tft.setCursor(85, 220);
      tft.println(pulsos_movimentacao_azimute);
    for (j = 0; j <= pulsos_movimentacao_azimute; j++) {
      MotorPasso_X.step(1);
      delay(10);
    }
  }
  else   {
    pulsos_movimentacao_azimute = ((abs (Azimute_ajustado - azim))*35) ;
      tft.setTextColor(WHITE, BLACK);
      tft.setTextSize(1);
      tft.setCursor(80, 205);
      tft.println(" Pulsos Moviv. Azimute    ");
      tft.setCursor(80, 220);
      tft.println("                          ");
      tft.setCursor(85, 220);
      tft.println(pulsos_movimentacao_azimute);
    hb =  (Azimute_ajustado - azim) / (abs (Azimute_ajustado - azim)) ;
    for (j = 0; j <= pulsos_movimentacao_azimute; j++) {
      MotorPasso_X.step(-hb);
      delay(10);
    }
  }
  Serial.print("Pulso Movimentacao Azimute: ");
  Serial.println(pulsos_movimentacao_azimute);

  
  azim = Azimute_ajustado;
  delay(1000);
  
}

void posicionarElevacao() {
  Serial.print("Elevevacao Anterior: ");
  Serial.println(elev);
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(1);
  tft.setCursor(40, 223);
  tft.println("     ");
  tft.setCursor(40, 223);
  tft.println(elev,1);

  tft.setTextColor(WHITE, BLUE);
  tft.setTextSize(2);
  tft.setCursor(150, 82);
  tft.println(alfa_elevacao,1);
  
  pulsos_movimentacao_elevacao = (elev - alfa_elevacao ) * 44 ; //Define o pulso da movimentacao da elevacao
  ha = pulsos_movimentacao_elevacao / (abs (pulsos_movimentacao_elevacao)); //Define o sentido da Elevacao
  Serial.print("Pulso Movimentacao Elevacao: ");
  Serial.println(pulsos_movimentacao_elevacao);
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(1);
  tft.setCursor(80, 205);
  tft.println(" Pulsos Moviv. Elevacao    ");
  tft.setCursor(80, 220);
  tft.println("                           ");
  tft.setCursor(85, 220);
  tft.println(pulsos_movimentacao_elevacao);
  
  for (i = 0; i <= (abs (pulsos_movimentacao_elevacao)); i++) {
    MotorPasso_Y.step(-ha);
    delay(10);
  }

  
  elev = alfa_elevacao;
  delay(1000);
}

void coletarSensores() {

  //valorSensor_pirel = 0;
  //valorSensor_piran = 0;

  static unsigned long delaycoleta;
  if ((millis() - delaycoleta) >= 200) {
  //PIRELIÔMETRO
  valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
  valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
  Serial.print("Valor Anlogico Pireliometro: ");
  Serial.println(valorSensor_pirel);
  tft.setTextColor(WHITE, BLUE);
  tft.setTextSize(2);
  tft.setCursor(150, 132);
  tft.println("    ");
  tft.setCursor(150, 132);
  tft.println(valorSensor_pirel);
  
  //PIRANÔMETRO
  valorSensor_piran = analogRead(sensor_piran); //ler os valores do pireliômetro
  valorSensor_piran = map (valorSensor_piran, 0, 1023, 0, 4000);
  Serial.print("Valor Anlogico Piranometro: ");
  Serial.println(valorSensor_piran);
  tft.setTextColor(WHITE, BLUE);
  tft.setTextSize(2);
  tft.setCursor(150, 107);
  tft.println("    ");
  tft.setCursor(150, 107);
  tft.println(valorSensor_piran);
  global_real = valorSensor_piran;// * (cos((90 -alfa_elevacao) * 0.0174533)));
  
  //RADIAÇÃO DIFUSA
  //delay(100);
  Rad_difusa = global_real - valorSensor_pirel ; //Valor da radiação difusa
  tft.setTextColor(WHITE, BLUE);
  tft.setTextSize(2);
  tft.setCursor(150, 157);
  tft.println("    ");
  tft.setCursor(150, 157);
  tft.println(Rad_difusa);

  delaycoleta = millis();
  }
  //delay (500); // delay para estabilizar
}


void printTela() {
  horaAtual();

  
//  Serial.println();
  // Serial.print (" Azimute calculado: "); Serial.println (Azimute_ajustado); Serial.print (" Elevacao calculado: "); Serial.println (alfa_elevacao);
  Serial.print (" Hora: "); Serial.print (h); Serial.print (":"); Serial.print (m); Serial.print (":"); Serial.println (s);
  Serial.print (" Rad Direta: "); Serial.println (valorSensor_pirel); Serial.print (" Rad global: "); Serial.println (global_real);
  Serial.print (" Rad difusa: "); Serial.println (Rad_difusa);
  
}


void zeramentoElevacao() {
  
  // Read normalized values   
  Vector normAccel = mpu.readNormalizeAccel();
  roll = (atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI;
 // Serial.print("Roll: ");
  //Serial.println(roll);
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(1);
  tft.setCursor(40, 223);
  tft.println(roll-1);
   delay(100);
   zer= ((roll - 1) * 44);
if (zer <0) {
   for (k=0; k<=- zer; k++) {
    MotorPasso_Y.step(1); //Anda sentido horario
    delay(10); //Frequencia de pulso para o motor
}
  delay(500);
  Vector normAccel = mpu.readNormalizeAccel();
  roll = (atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI;
  Serial.println(roll);
  delay(3000);
  zer= ((roll - 1) * 44);
    if (zer <0) {
   for (k=0; k<=- zer; k++) {
    MotorPasso_Y.step(1); //Anda sentido horario
    delay(20); //Frequencia de pulso para o motor
}
    }
    else {
   for (k=0; k<=zer; k++) {
    MotorPasso_Y.step(-1); //Anda sentido horario
    delay(20); //Frequencia de pulso para o motor
}

}
}

else {
   for (k=0; k<=zer; k++) {
    MotorPasso_Y.step(-1); //Anda sentido horario
    delay(10); //Frequencia de pulso para o motor
}
delay(500);
  Vector normAccel = mpu.readNormalizeAccel();
  roll = (atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI;
  Serial.println(roll);
  delay(3000);
  zer= ((roll - 1) * 44);
    if (zer <0) {
   for (k=0; k<=- zer; k++) {
    MotorPasso_Y.step(1); //Anda sentido horario
    delay(20); //Frequencia de pulso para o motor
}
    }
    else {
   for (k=0; k<=zer; k++) {
    MotorPasso_Y.step(-1); //Anda sentido horario
    delay(20); //Frequencia de pulso para o motor
}

}
}
estadoCodigo = 1;
  Serial.println ("Zeramento de elevacao OK");
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(1);
  tft.setCursor(80, 205);
  tft.println("                          ");
  tft.setCursor(80, 205);
  tft.println(" Zeramento de elevacao OK ");
  tft.setCursor(80, 220);
  tft.println("                          ");
  tft.setCursor(40, 223);
  tft.println(" 0   ");
  tft.setTextSize(2);
  tft.setCursor(280, 215);
  tft.println(estadoCodigo);
  tft.setTextColor(WHITE, BLUE);
  tft.setTextSize(2);
  tft.setCursor(160, 82);
  tft.println(" 0   ");
}

void zeramentoPosicao() {
  Texto_Data();
  qmc.read(&qmcx, &qmcy, &qmcz, &azimuth); //Print teste posicao
  int increment_azim = 0;
 int  media_azim=0;
 int kp;
  for (i = 0; i <=4; i++) {
  increment_azim += azimuth;
  delay(200);
  //Serial.println(increment_azim);
  }
  media_azim = increment_azim/5;
  Serial.print("Media Azim ");Serial.println(media_azim);
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(1);
  tft.setCursor(40, 210);
  tft.println("     ");
  tft.setCursor(40, 210);
  tft.println(media_azim);
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(1);
  tft.setCursor(80, 205);
  tft.println(" Realizando zeramento Pos ");
  tft.setCursor(80, 220);
  tft.println(" SetPoint = 143,   Pos = 0");
  if (media_azim < 143 ) { //Testa se está no limite inferior
    //qmc.read(&qmcx, &qmcy, &qmcz,&azimuth); //Print teste posicao
    for ( kp = 0; kp <=35; kp++) {
    MotorPasso_X.step(-1); //Anda sentido horario
    delay(20); //Frequencia de pulso para o motor
    }
   
  }

  
  else if ( media_azim > 144) { //Testa se está no limite superior
    //qmc.read(&qmcx, &qmcy, &qmcz, &azimuth);//Print teste posicao
    for ( kp = 0; kp <=35; kp++) {
    MotorPasso_X.step(1); //Anda sentido horario
    delay(20); //Frequencia de pulso para o motor
    }
    
  }
 // Serial.print("Azimuth: ");
 // Serial.println(azimuth);
  else if (media_azim == 143) {
    estadoCodigo = 2;
      Serial.println ("Zeramento de azimute OK");
        
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(1);
  tft.setCursor(80, 205);
  tft.println("                          ");
  tft.setCursor(80, 205);
  tft.println(" Zeramento de Azimute OK  ");
  tft.setCursor(80, 220);
  tft.println("                          ");
  tft.setCursor(40, 210);
  tft.println(" 0  ");
  tft.setTextSize(2);
  tft.setCursor(280, 215);
  tft.println(estadoCodigo);
  tft.setTextColor(WHITE, BLUE);
  tft.setTextSize(2);
  tft.setCursor(160, 57);
  tft.println(" 0   ");
  }
}


void armazenarSD() {
   dataFile = SD.open(Nome_Arq, FILE_WRITE);
  if(dataFile) {
    if (dia < 10)
  {
    dataFile.print("0");
  }
        dataFile.print(dia); dataFile.print( ", ");
        if (mes < 10)
  {
    dataFile.print("0");
  }
        dataFile.print(mes); dataFile.print( ", ");
        dataFile.print(ano); dataFile.print( ", ");
        if (h < 10)
  {
    dataFile.print("0");
  }
        dataFile.print(h); dataFile.print( ", ");
         if (m < 10)
  {
    dataFile.print("0");
  }
        dataFile.print(m); dataFile.print( ", ");
        if (s < 10)
  {
    dataFile.print("0");
  }
        dataFile.print(s); dataFile.print( ", ");
        dataFile.print(valorSensor_pirel); dataFile.print( ", ");
        dataFile.print(global_real); dataFile.print( ", ");
        dataFile.println(Rad_difusa);
 
        delay(100);
    dataFile.close(); // fecha o arquivo
  }
  
  else {
    Serial.println("Erro ao abrir arquivo para escrita final");
  }
}

void decisao_refinamento() {

   delay(200);
  valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
  valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
  int increment_pirel = 0;
  int  media_pirel=0;
  int kp;
  for (kp = 0; kp <=4; kp++) {
  increment_pirel += valorSensor_pirel;
  delay(200);
  Serial.println(increment_pirel);
  }
  media_pirel = increment_pirel/5;
  Serial.println(media_pirel);


  valorSensor_piran = analogRead(sensor_piran); //ler os valores do pireliômetro
  valorSensor_piran = map (valorSensor_piran, 0, 1023, 0, 4000);

  int increment_piran = 0;
  int  media_piran=0;
  int ki;
  for (ki = 0; ki <=4; ki++) {
  increment_piran += valorSensor_piran;
  delay(200);
  Serial.println(increment_piran);
  }
  media_piran = increment_piran/5;
  Serial.println(media_piran);

  
  tft.setTextColor(WHITE, BLUE);
  tft.setTextSize(2);
  tft.setCursor(150, 132);
  tft.println("      ");
  tft.setCursor(150, 132);
  tft.println(media_pirel);

  tft.setTextColor(WHITE, BLUE);
  tft.setTextSize(2);
  tft.setCursor(150, 107);
  tft.println("      ");
  tft.setCursor(150, 107);
  tft.println(media_piran);
  delay(2000);
  
 if ((media_pirel < 80) && (media_piran > 400) ) {
  estadoCodigo =4;
    
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(2);
    tft.setCursor(280, 215);
    tft.println(estadoCodigo);

  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(1);
  tft.setCursor(80, 205);
  tft.println(" Modo Manual (Controle)   ");
  tft.setCursor(80, 220);
  tft.println("                          ");
  tft.setCursor(80, 220);
  tft.println("                          ");

  
 }

else {
    estadoCodigo =5;

    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(2);
    tft.setCursor(280, 215);
    tft.println(estadoCodigo);

  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(1);
  tft.setCursor(80, 205);
  tft.println(" Refinamento Automatico   ");
  tft.setCursor(80, 220);
  tft.println("                          ");
  tft.setCursor(80, 220);
  tft.println("                          ");

}
delay(1000);
}
 void refinamento_automatico(){
  valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
  valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
  myMovingAverage_1 += valorSensor_pirel;
  myMovingAverage_1.process();
  filtred_pirel = myMovingAverage_1.mean;
  Direta_ref = filtred_pirel;

  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(1);
  tft.setCursor(80, 205);
  tft.println("                          ");
  tft.setCursor(80, 205);
  tft.println(" Ajuste Automatico Azimute");
  tft.setCursor(80, 220);
  tft.println("                          ");
  tft.setCursor(80, 220);
  tft.println("                          ");
  
  if ((Direta_ref - ajuste_azim_1) > tolerancia) {
    for (j = 0; j <= 6; j++) {
      MotorPasso_X.step(1); // + 1 GRAU PARA DIREITA
      delay(10);

    }
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(1);
    tft.setCursor(80, 220);
    tft.println("                          ");
    tft.setCursor(80, 220);
    tft.println(" Ajuste para a Direita    ");
    delay(2000);
    valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
    valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
    myMovingAverage_1 += valorSensor_pirel;
    myMovingAverage_1.process();
    filtred_pirel = myMovingAverage_1.mean;
    Serial.print ("  Rad Direta: "); Serial.println (filtred_pirel);
    tft.setTextColor(WHITE, BLUE);
    tft.setTextSize(2);
    tft.setCursor(150, 132);
    tft.println("    ");
    tft.setCursor(150, 132);
    tft.println(filtred_pirel);
    ajuste_azim_1 = filtred_pirel;

    for (j = 0; j <= 6; j++) {
      MotorPasso_X.step(1); // + 1 GRAU PARA DIREITA
      delay(10);

    }
    delay(2000);
  }

  else if ((ajuste_azim_1 - Direta_ref) > tolerancia)  {
    for (j = 0; j <= 12; j++) {
      MotorPasso_X.step(-1); // + 1 GRAU PARA ESQUERDA
      delay(10);
      /*valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
      valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
      myMovingAverage_1 += valorSensor_pirel;
      myMovingAverage_1.process();
      filtred_pirel = myMovingAverage_1.mean;
      */
      //Serial.print ("  Rad Direta: "); Serial.println (filtred_pirel);
    }
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(1);
    tft.setCursor(80, 220);
    tft.println("                          ");
    tft.setCursor(80, 220);
    tft.println(" Ajuste para a Esquerda   ");
    delay(2000);
    valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
    valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
    myMovingAverage_1 += valorSensor_pirel;
    myMovingAverage_1.process();
    filtred_pirel = myMovingAverage_1.mean;
    ajuste_azim_1 = filtred_pirel;
    Serial.print ("  Rad Direta: "); Serial.println (filtred_pirel);
    tft.setTextColor(WHITE, BLUE);
    tft.setTextSize(2);
    tft.setCursor(150, 132);
    tft.println("    ");
    tft.setCursor(150, 132);
    tft.println(filtred_pirel);
    for (j = 0; j <= 12; j++) {
      MotorPasso_X.step(-1); // + 1 GRAU PARA DIREITA
      delay(10);
    }
    delay(2000);
  }

  else   {

    valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
    valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
    myMovingAverage_1 += valorSensor_pirel;
    myMovingAverage_1.process();
    filtred_pirel = myMovingAverage_1.mean;
    Serial.print("seguidor refinado____________ valor da radiação: "); Serial.println(filtred_pirel);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(1);
    tft.setCursor(80, 220);
    tft.println("                          ");
    tft.setCursor(80, 220);
    tft.println(" Seguidor Refinado Azimute");
      
      estadoCodigo = 6;
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(2);
  tft.setCursor(280, 215);
  tft.println("  ");
  tft.setCursor(280, 215);
  tft.println(estadoCodigo);
  }

}

void refinamento_automatico_elev(){
  valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
  valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
  myMovingAverage_1 += valorSensor_pirel;
  myMovingAverage_1.process();
  filtred_pirel = myMovingAverage_1.mean;
  Direta_ref = filtred_pirel;

  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(1);
  tft.setCursor(80, 205);
  tft.println("                          ");
  tft.setCursor(80, 205);
  tft.println("Ajuste Automatico Elevacao");
  tft.setCursor(80, 220);
  tft.println("                          ");
  tft.setCursor(80, 220);
  tft.println("                          ");
  
  if ((Direta_ref - ajuste_azim_1) > tolerancia) {
    for (j = 0; j <= 8; j++) {
      MotorPasso_Y.step(1); // + 1 GRAU PARA DIREITA
      delay(10);

    }
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(1);
    tft.setCursor(80, 220);
    tft.println("                          ");
    tft.setCursor(80, 220);
    tft.println(" Ajuste para a Baixo      ");
    delay(2000);
    valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
    valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
    myMovingAverage_1 += valorSensor_pirel;
    myMovingAverage_1.process();
    filtred_pirel = myMovingAverage_1.mean;
    Serial.print ("  Rad Direta: "); Serial.println (filtred_pirel);
    tft.setTextColor(WHITE, BLUE);
    tft.setTextSize(2);
    tft.setCursor(150, 132);
    tft.println("    ");
    tft.setCursor(150, 132);
    tft.println(filtred_pirel);
    
    ajuste_azim_1 = filtred_pirel;

    for (j = 0; j <= 8; j++) {
      MotorPasso_Y.step(1); // + 1 GRAU PARA DIREITA
      delay(10);

    }
    delay(2000);
  }

  else if ((ajuste_azim_1 - Direta_ref) > tolerancia)  {
    for (j = 0; j <= 16; j++) {
      MotorPasso_Y.step(-1); // + 1 GRAU PARA ESQUERDA
      delay(10);
      /*
      valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
      valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
      myMovingAverage_1 += valorSensor_pirel;
      myMovingAverage_1.process();
      filtred_pirel = myMovingAverage_1.mean;
      Serial.print ("  Rad Direta: "); Serial.println (filtred_pirel);
      */
    }
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(1);
    tft.setCursor(80, 220);
    tft.println("                          ");
    tft.setCursor(80, 220);
    tft.println(" Ajuste para a Cima       ");
    delay(2000);
    valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
    valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
    myMovingAverage_1 += valorSensor_pirel;
    myMovingAverage_1.process();
    filtred_pirel = myMovingAverage_1.mean;
    Serial.print ("  Rad Direta: "); Serial.println (filtred_pirel);
    tft.setTextColor(WHITE, BLUE);
    tft.setTextSize(2);
    tft.setCursor(150, 132);
    tft.println("    ");
    tft.setCursor(150, 132);
    tft.println(filtred_pirel);
    
    ajuste_azim_1 = filtred_pirel;

    for (j = 0; j <= 16; j++) {
      MotorPasso_Y.step(-1); // + 1 GRAU PARA DIREITA
      delay(10);
    }
    delay(2000);
  }

  else   {

    valorSensor_pirel = analogRead(sensor_pirel); //ler os valores do pireliômetro
    valorSensor_pirel = map (valorSensor_pirel, 0, 1023, 0, 4000);
    myMovingAverage_1 += valorSensor_pirel;
    myMovingAverage_1.process();
    filtred_pirel = myMovingAverage_1.mean;
    Serial.print("seguidor refinado____________ valor da radiação: "); Serial.println(filtred_pirel);
      tft.setTextColor(WHITE, BLACK);
      tft.setTextSize(1);
      tft.setCursor(80, 220);
      tft.println("                          ");
      tft.setCursor(80, 220);
      tft.println("Seguidor Refinado Elevacao");
      
      estadoCodigo = 7;
      tft.setTextColor(WHITE, BLACK);
      tft.setTextSize(2);
      tft.setCursor(280, 215);
      tft.println("  ");
      tft.setCursor(280, 215);
      tft.println(estadoCodigo);
  }

}

void teste_linhas(uint16_t color)
{
  tft.fillScreen(BLACK);
  for (uint16_t x = 0; x < tft.width(); x += 6)
  {
    tft.drawLine(0, 0, x, tft.height() - 1, color);
  }
  for (uint16_t y = 0; y < tft.height(); y += 6)
  {
    tft.drawLine(0, 0, tft.width() - 1, y, color);
  }
}
  
void teste_retangulo_cheio(uint16_t color1, uint16_t color2)
{
  tft.fillScreen(BLACK);
  for (uint16_t x = tft.width() - 1; x > 6; x -= 6)
  {
    tft.fillRect(tft.width() / 2 - x / 2, tft.height() / 2 - x / 2 , x, x, color1);
    tft.drawRect(tft.width() / 2 - x / 2, tft.height() / 2 - x / 2 , x, x, color2);
  }
}
  
void teste_circulos(uint8_t radius, uint16_t color)
{
  for (uint16_t x = radius; x < tft.width(); x += radius * 2)
  {
    for (uint16_t y = radius; y < tft.height(); y += radius * 2) {
      tft.fillCircle(x, y, radius, color);
    }
  }
  for (uint16_t x = 0; x < tft.width() + radius; x += radius * 2)
  {
    for (uint16_t y = 0; y < tft.height() + radius; y += radius * 2)
    {
      tft.drawCircle(x, y, radius, WHITE);
    }
  }
}
  
void teste_triangulos()
{
  tft.fillScreen(BLACK);
  for (uint16_t i = 0; i < tft.width() / 2; i += 5)
  {
    tft.drawTriangle(tft.width() / 2, tft.height() / 2 - i,
                     tft.width() / 2 - i, tft.height() / 2 + i,
                     tft.width() / 2 + i, tft.height() / 2 + i, tft.Color565(0, 0, i));
  }
}
  
void teste_retangulos()
{
  tft.fillScreen(BLACK);
  for (uint16_t x = tft.width(); x > 20 ; x -= 6)
  {
    tft.fillRoundRect(tft.width() / 2 - x / 2, tft.height() / 2 - x / 2 , x, x, x / 8,  tft.Color565(0, x, 0));
  }
}

void apresentacao() {
  tft.setRotation(1);
  tft.fillScreen(WHITE);
  tft.setTextColor(BLACK);
  tft.setTextSize(4);
  tft.setCursor(20, 15);
  tft.println("UNIVESIDADE ");
  tft.setCursor(30, 65);
  tft.println("FEDERAL DE");  
  tft.setCursor(55, 115);
  tft.println("SERGIPE");
  tft.setCursor(30, 165);
  tft.println("DMEC - UFS");
  tft.setTextSize(1);
  tft.setCursor(5, 200);
  tft.println("Orientador: Dr. Douglas Bressan Riffel");
  tft.setCursor(5, 220);
  tft.println("Graduando: Jose Orlando ROdrigues Santos");
  delay(3000);
  teste_linhas(CYAN);
  delay(500);
}

void printStaticTela()
{ 
  printCabecalhoRet();
  TextoCabecalho();
  printCelulaRet();

  printAmbienteRet();
  TextoAmbiente();
}

void printCabecalhoRet() {
  tft.fillRoundRect(2, 2, 316,24 , 5, BLACK);
  tft.drawRoundRect(2, 2, 316,24 , 5, WHITE);
  }
  
  void TextoCabecalho() {
  tft.setRotation(1);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.setCursor(6, 6);
  tft.println("UNIVERSIDADE FEDERAL DE SERGIPE UFS Data: ");
  tft.setCursor(6, 16);
  tft.println("DMEC - GRADUANDO: JOSE ORLANDO      Hora: ");
  }

void printCelulaRet() {
  tft.fillRoundRect(2, 28, 316,160 , 5, BLUE);
  tft.drawRoundRect(2, 28, 316,160 , 5, WHITE);
  tft.fillRoundRect(25, 32, 276, 22 , 5, GREEN);
  tft.drawRoundRect(25, 32, 276, 22 , 5, WHITE);
  

}

void printAmbienteRet() {
  
  tft.fillRoundRect(2, 190, 316, 46, 5, BLACK);
  tft.drawRoundRect(2, 190, 316, 46, 5, WHITE);
  tft.fillRoundRect(75, 187, 170, 49 , 5, BLACK);
  tft.drawRoundRect(75, 187, 170, 49 , 5, GREEN);
  tft.fillRoundRect(90, 176, 140, 22 , 5, GREEN);
  tft.drawRoundRect(90, 176, 140, 22 , 5, WHITE);
}

void TextoAmbiente() {
  
  tft.setRotation(1);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.setCursor(5, 195);
  tft.println("Posicao Ant");
  tft.setCursor(10, 210);
  tft.println("Azim: ");
  tft.setCursor(10, 223);
  tft.println("Elev: ");
  
  tft.setTextColor(BLACK, GREEN);
  tft.setTextSize(2);
  tft.setCursor(95, 180);
  tft.println("Informacoes");
  
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.setCursor(270, 195);
  tft.println("  Est. ");
  tft.setCursor(270, 205); 
  tft.println("Codigo:");
  
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(60, 35);
  tft.println("Modo:");
  tft.setTextColor(WHITE);
  tft.setCursor(10, 57);
  tft.println("Azimuth: ");
  tft.setCursor(225, 57);
  tft.print((char)247);
  tft.setCursor(10, 82);
  tft.println("Elevacao: ");
  tft.setCursor(225, 82);
  tft.print((char)247);
  tft.setCursor(10, 107);
  tft.println("GHI: ");
  tft.setCursor(225, 107);
  tft.print("W/m^2");
  tft.setCursor(10, 132);
  tft.println("DNI: ");
  tft.setCursor(225, 132);
  tft.print("W/m^2");
  tft.setCursor(10, 157);
  tft.println("G_DIf: ");
  tft.setCursor(225, 157);
  tft.print("W/m^2");
}

void Texto_Data() {
  horaAtual();
static unsigned long delaydata;

 if ((millis() - delaydata) >= 999) {
tft.setRotation(1);

 tft.setTextSize(1);
  tft.setTextColor(GREEN, BLACK);
  tft.setCursor(252, 6);
  if (dia < 10)
  {
     
     tft.print("0");
  }
  tft.print(dia);
  tft.print("/");
  if (mes < 10)
  {
     tft.print("0");
  }
  tft.print(mes);
  tft.print("/");

  tft.println(ano);
  tft.setTextSize(1);
  tft.setTextColor(GREEN, BLACK);
 tft.setCursor(252, 16);
  if (h < 10)
  {
     
     tft.print("0");
  }
  tft.print(h);
  tft.print(":");
  if (m < 10)
  {
     tft.print("0");
  }
  tft.print(m);
  tft.print(":");
    if (s < 10)
  {
     tft.print("0");
  }
  tft.print(s);

delaydata = millis();
  }
  
}

void controle_manual() {
  
error = 0; 
type = 1;

static int contx =0;
static int conty =0;


coletarSensores();
ps2x.read_gamepad(false, vibrate); 

 static unsigned long tempo_delay = 0;
 if ((millis() - tempo_delay) >= 25000) {
  
  tempo_delay = millis();
  estadoCodigo=7;
  contx = 0;
  conty = 0;
 }
  
/*
if(ps2x.Analog(PSS_LY) > 130 && ps2x.Analog(PSS_LY) <= 255) // Se o botão analógico L for pressionado para direita (eixo X).
{

    MotorPasso_Y.step(-1);
    delay(10);
 } 
if(ps2x.Analog(PSS_LY) >= 0 && ps2x.Analog(PSS_LY) <= 126) // Se o botão analógico L for pressionado para esquerda (eixo X)
{
    MotorPasso_Y.step(1);
    delay(10);  
 }

 if(ps2x.Analog(PSS_RX) > 130 && ps2x.Analog(PSS_RX) <= 255) // Se o botão analógico L for pressionado para direita (eixo X).
{

    MotorPasso_X.step(1);
    delay(10);
 } 
if(ps2x.Analog(PSS_RX) >= 0 && ps2x.Analog(PSS_RX) <= 126) // Se o botão analógico L for pressionado para esquerda (eixo X)
{
    MotorPasso_X.step(-1);
    delay(10);  
 }
*/
 
 if(ps2x.ButtonPressed(PSB_RED))     {         //se esse botao (circulo) for pressionado avança para o estado codigo 6
         Serial.println("estadoCodigo = 7 ");
         estadoCodigo=7;
         //delay(100);
         contx = 0;
         conty = 0;
         tft.setTextColor(WHITE, BLACK);
         tft.setTextSize(1);
         tft.setCursor(80, 220);
         tft.println("                          ");
         tft.setCursor(80, 220);
         tft.println(" Retorno Coleta de Dados  ");
 }
  if(ps2x.ButtonPressed(PSB_PINK))    {         //se esse botao (circulo) for pressionado avança para o estado codigo 6
         k = 1;
         Serial.println("estadoCodigo = 4 ");
         estadoCodigo=4;
         tempo_delay = millis();
         //delay(100);
         tft.setTextColor(WHITE, BLACK);
         tft.setTextSize(1);
         tft.setCursor(80, 220);
         tft.println("                          ");
         tft.setCursor(80, 220);
         tft.println(" Modo Manual Acionado     ");
 }
  if(ps2x.Button(PSB_PAD_RIGHT)){
    contx ++;
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(1);
    tft.setCursor(80, 220);
    tft.println("                          ");
    tft.setCursor(80, 220);
    tft.print("Azimute: ");tft.print(contx);tft.print("  Elevacao: ");tft.print(conty);
      for (j = 0; j <= 34; j++) {
      MotorPasso_X.step(1);
      delay(20);
    }
    
    
    tempo_delay = millis();
    //delay(100);
 }

  if(ps2x.Button(PSB_PAD_LEFT)){
    contx --;
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(1);
    tft.setCursor(80, 220);
    tft.println("                          ");
    tft.setCursor(80, 220);
    tft.print("Azimute: ");tft.print(contx);tft.print("  Elevacao: ");tft.print(conty);
      for (j = 0; j <= 34; j++) {
      MotorPasso_X.step(-1);
      delay(20);
    }
    
   
    tempo_delay = millis();
    //delay(100);
 }
  if(ps2x.Button(PSB_PAD_UP)){
    conty ++;
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(1);
    tft.setCursor(80, 220);
    tft.println("                          ");
    tft.setCursor(80, 220);
    tft.print("Azimute: ");tft.print(contx);tft.print("  Elevacao: ");tft.print(conty);
      for (j = 0; j <= 44; j++) {
      MotorPasso_Y.step(1);
      delay(25);
    }
    
    
    tempo_delay = millis();
    //delay(100);
 }

  if(ps2x.Button(PSB_PAD_DOWN)){
    conty --;
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(1);
    tft.setCursor(80, 220);
    tft.println("                          ");
    tft.setCursor(80, 220);
    tft.print("Azimute: ");tft.print(contx);tft.print("  Elevacao: ");tft.print(conty);
      for (j = 0; j <= 44; j++) {
      MotorPasso_Y.step(-1);
      delay(25);
    }
    
    
    tempo_delay = millis();
    //delay(100);
 }
  if(ps2x.Button(PSB_L3)) {
         tft.setTextColor(WHITE, BLACK);
         tft.setTextSize(1);
         tft.setCursor(80, 220);
         tft.println("                          ");
         tft.setCursor(80, 220);
         tft.println(" Zeramento de Elevacao    ");
    zeramentoElevacao();
   
    tempo_delay = millis();
  }
  if(ps2x.Button(PSB_R3)) {
         tft.setTextColor(WHITE, BLACK);
         tft.setTextSize(1);
         tft.setCursor(80, 220);
         tft.println("                          ");
         tft.setCursor(80, 220);
         tft.println(" Zeramento de Posicao     ");
    zeramentoPosicao();
    tempo_delay = millis();
  }
  if(ps2x.Button(PSB_L1)) {
    contx = contx-5;
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(1);
    tft.setCursor(80, 220);
    tft.println("                          ");
    tft.setCursor(80, 220);
    tft.print("Azimute: ");tft.print(contx);tft.print("  Elevacao: ");tft.print(conty);
    
    for (j = 0; j <= (5 * 34); j++) {
      MotorPasso_X.step(-1);
      delay(20);
    }
    
    tempo_delay = millis();
  }
  if (ps2x.Button(PSB_R1)) {
    contx = contx+5;
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(1);
    tft.setCursor(80, 220);
    tft.println("                          ");
    tft.setCursor(80, 220);
    tft.print("Azimute: ");tft.print(contx);tft.print("  Elevacao: ");tft.print(conty);

      for (j = 0; j <= (5 * 34); j++) {
      MotorPasso_X.step(1);
      delay(20);
    }
        tempo_delay = millis();
  }
   
}
