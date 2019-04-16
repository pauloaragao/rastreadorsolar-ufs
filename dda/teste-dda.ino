int dia, mes,ano;
int dda;


void calculoDDA(){
   if (ano % 4 == 0 && (ano % 400 == 0 || ano % 100 != 0)) {
    switch (mes) {
      case 1:
        dda = dia;
        break;
      case 2:
        dda = dia + 31;
        break;
      case 3:
        dda = dia + 60;
        break;
      case 4:
        dda = dia + 91;
        break;
      case 5:
        dda = dia + 121;
        break;
      case 6:
        dda = dia + 152;
        break;
      case 7:
        dda = dia + 182;
        break;
      case 8:
        dda = dia + 213;
        break;
      case 9:
        dda = dia + 244;
        break;
      case 10:
        dda = dia + 274;
        break;
      case 11:
        dda = dia + 305;
        break;
      case 12:
        dda = dia + 335;
        break;
      default:
        break;
    }
  }

  else {
    switch (mes) {
      case 1:
        dda = dia;
        break;
      case 2:
        dda = dia + 31;
        break;
      case 3:
        dda = dia + 59;
        break;
      case 4:
        dda = dia + 90;
        break;
      case 5:
        dda = dia + 120;
        break;
      case 6:
        dda = dia + 151;
        break;
      case 7:
        dda = dia + 181;
        break;
      case 8:
        dda = dia + 212;
        break;
      case 9:
        dda = dia + 243;
        break;
      case 10:
        dda = dia + 273;
        break;
      case 11:
        dda = dia + 304;
        break;
      case 12:
        dda = dia + 334;
        break;
      default:
        break;
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  dia = 15;
  mes = 10;
  ano = 2012;
}

void loop() {
  // put your main code here, to run repeatedly:
  calculoDDA();
  Serial.println(dda);
  
}
