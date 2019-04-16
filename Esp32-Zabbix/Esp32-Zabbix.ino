/*
 *  This sketch sends data via HTTP GET requests to data.sparkfun.com service.
 *
 *  You need to get streamId and privateKey at data.sparkfun.com and paste them
 *  below. Or just customize this script to talk to other HTTP servers.
 *
 */

#include <WiFi.h>

const char* ssid     = "DMEC_ALUNOS";
const char* password = "31415925";

const char* host = "data.sparkfun.com";
const char* streamId   = "....................";
const char* privateKey = "....................";

//Server do Zabbix
  const char* server = "";
  int porta = 10051; //Porta do Proctocolo

//Inicializando a comunicação do servidor
  WiFiClient client;

//Configurando intervalos de medidas:
  long previousMillis = 0;
  long interval = 10000;


void config_inicial(){
  Serial.begin(115200); //Configurando a UART

    Serial.print("Conectando na rede: ");
    Serial.println(ssid);

    WiFi.begin(ssid,password);


    while(WiFi.status() != WL_CONNECTED){
      delay(500);
      Serial.print(".");
    }

    Serial.print("Conectado na rede local: ");
    Serial.println(ssid);
    Serial.println("IP: ");
    Serial.println(WiFi.localIP());
    Serial.println()
    
  
  
}

 
void setup()
{
   config_inicial();
}

int value = 0;

void loop()
{
    delay(5000);
    ++value;

    Serial.print("connecting to ");
    Serial.println(host);

    // Use WiFiClient class to create TCP connections
    WiFiClient client;
    const int httpPort = 80;
    if (!client.connect(host, httpPort)) {
        Serial.println("connection failed");
        return;
    }

    // We now create a URI for the request
    String url = "/input/";
    url += streamId;
    url += "?private_key=";
    url += privateKey;
    url += "&value=";
    url += value;

    Serial.print("Requesting URL: ");
    Serial.println(url);

    // This will send the request to the server
    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Connection: close\r\n\r\n");
    unsigned long timeout = millis();
    while (client.available() == 0) {
        if (millis() - timeout > 5000) {
            Serial.println(">>> Client Timeout !");
            client.stop();
            return;
        }
    }

    // Read all the lines of the reply from server and print them to Serial
    while(client.available()) {
        String line = client.readStringUntil('\r');
        Serial.print(line);
    }

    Serial.println();
    Serial.println("closing connection");
}
