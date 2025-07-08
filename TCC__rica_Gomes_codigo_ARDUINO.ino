/*
  Centro Federal de Educação Tecnólogica de Minas Gerais
  Trabalho de Conclusão de Curso II
  Projeto: Sistema de Telegestão para Iluminação Publíca
  Autor: Érica Gomes Fernandes
  Data: 08/07/2025
*/

// === BIBLIOTECAS NECESSÁRIAS ===
#include <WiFi.h>              // Conexão com rede Wi-Fi
#include <PubSubClient.h>      // Comunicação MQTT
#include <ArduinoJson.h>       // Manipulação de dados em formato JSON
#include <ZMPT101B.h>          // Sensor de tensão 
#include <TinyGPSPlus.h>       // Leitura de dados do GPS
#include <HardwareSerial.h>    // Comunicação serial adicional com GPS
#include "EmonLib.h"           // Biblioteca para sensor de corrente SCT-013

// === DEFINIÇÕES DE PINOS E VARIÁVEIS ===
#define SAMPLES 1000                 // Número de amostras 
#define pino_sensor 35              // Pino analógico para ZMPT101B

// Vetor e variáveis para média móvel da corrente
float vet[11] = {0};                // Vetor circular para suavizar o valor de corrente
float CorrenteMedia = 0;           
float sum = 0;                     // Soma das amostras
float Irms_medio2 = 0;             // Corrente RMS filtrada

EnergyMonitor SCT013;              // Objeto da biblioteca EmonLib para medir corrente
int pinSCT = 34;                   // Pino conectado ao SCT-013

// Calibração do sensor de tensão ZMPT101B
float offset = 2900;               // Offset do sensor (valor de referência)
float fator_calibracao = 0.695;    // Fator de correção da tensão
int numero_amostras = 500;         // Total de amostras para cálculo RMS

// === GPS ===
TinyGPSPlus gps;                   // Objeto do GPS
HardwareSerial gpsSerial(1);      // Serial1 para comunicação com GPS (RX=16, TX=17)

// === CONSTANTES ===
const float sensibilidade = 0.185;   // Sensibilidade do sensor de corrente 
const float tensaoReferencia = 2.5;  // Tensão de referência 

int tensao = 0;     // Variáveis auxiliares (não usadas)
int corrente = 0;

// === MQTT E REDE ===
String clientID = "ESP32";                    // ID do cliente MQTT
const char *mqtt_server = "broker.hivemq.com";// Endereço do broker público MQTT
const char *topic = "Tempdata";               // Tópico para publicação

// Credenciais da rede Wi-Fi
const char *ssid = "************";
const char *password = "*********";

// Objetos para conexão
WiFiClient espClient;
PubSubClient client(espClient);

byte led = 4;                           // Pino do LED de controle
unsigned long previousMillis = millis(); // Controle de tempo

// === FUNÇÃO: RECONEXÃO COM MQTT ===
void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    clientID += String(random(0xffff), HEX); // Gera ID aleatório se necessário

    if (client.connect(clientID.c_str()))
    {
      Serial.println("connected");

      // Inscrição nos tópicos do Node-RED
      client.subscribe("fromNodeRED");
      client.subscribe("ledState");
      client.subscribe("JSONfromNode");
      client.subscribe("controle");
      client.subscribe("controle1");
      client.subscribe("controle2");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000); // Aguarda 5 segundos antes de tentar novamente
    }
  }
}

// === FUNÇÃO: CALLBACK DO MQTT ===
void callback(char *topic, byte* message, unsigned int length)
{
  Serial.print("Message arrived :");
  Serial.print(topic);
  Serial.print(". Message ");

  String messageTemp;

  // Monta a string da mensagem
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }

  // Ações com base no tópico recebido
  if (String(topic) == "fromNodeRED")
  {
    Serial.println(messageTemp);
  }

  else if (String(topic) == "ledState")
  {
    if (messageTemp == "on")
    {
      digitalWrite(led, LOW);  // Liga o LED (nível baixo no ESP32)
    }
    else if (messageTemp == "off")
    {
      digitalWrite(led, HIGH); // Desliga o LED
    }
  }

  else if (String(topic) == "JSONfromNode")
  {
    // Deserializa a string JSON recebida
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, messageTemp);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    // Lê os valores do JSON (não estão sendo usados no código)
    int tensao_rms = doc["tensao"];
    int Irms_medio2 = doc["corrente"];
  }

  else if (String(topic) == "controle")
  {
    // Controle PWM com valor recebido
    String valor = messageTemp;
    analogWrite(4, valor.toInt());
  }

  else if (String(topic) == "controle1")
  {
    analogWrite(4, 250);
  }

  else if (String(topic) == "controle2")
  {
    analogWrite(4, 0);
  }
}

// === FUNÇÃO: CONECTAR AO WI-FI ===
void connectAP()
{
  Serial.println("Connect to my WiFi");
  WiFi.begin(ssid, password);
  byte cnt = 0;

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
    cnt++;
  }
}

// === SETUP: CONFIGURAÇÕES INICIAIS ===
void setup()
{
  Serial.begin(115200);           // Inicia comunicação serial
  pinMode(led, OUTPUT);           // LED como saída
  analogReadResolution(12);       // Define resolução de leitura ADC (ESP32 usa 12 bits)
  connectAP();                    // Conecta ao Wi-Fi

  client.setServer(mqtt_server, 1883);   // Configura servidor MQTT
  client.setCallback(callback);         // Define função de callback para mensagens

  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // Inicia comunicação com GPS
  SCT013.current(pinSCT, 2.9586);           // Configura calibração do sensor de corrente
  pinMode(pino_sensor, INPUT);             // Pino do sensor de tensão como entrada

  Serial.println("Iniciando GPS...");
}

// === FUNÇÃO: MEDIR CORRENTE COM SCT-013 ===
void medirCorrenteAC() 
{
   double Irms = SCT013.calcIrms(1480);   // Mede a corrente em tempo real

   // Calcula média móvel com as 11 últimas leituras
   for(int n = 0; n < 10; n++)
  {
    vet[n] = vet[n+1];
    sum = sum + vet[n];
  }  

  vet[10] = Irms;
  float Irms_medio = (sum + vet[10]) / 11;
  sum = 0;

  // Ajuste baseado em calibração experimental
  Irms_medio2 = 0.2518 * Irms_medio - 0.018;
    
  Serial.print("Corrente = ");
  Serial.print(Irms_medio2);
  Serial.println(" A");   
    
  // Envia valor por MQTT
  client.publish("corrente", String(Irms_medio2).c_str());
    
  delay(300); // Pequena pausa
}

// === FUNÇÃO: MEDIR TENSÃO COM ZMPT101B ===
void medirTensao()
{
  float soma_quadrados = 0;
  float valor_analogico;
  float valor_lido;

  // Captura de amostras para cálculo RMS
  for (int i = 0; i < numero_amostras; i++) {
    valor_analogico = analogRead(pino_sensor);
    valor_lido = valor_analogico - offset;
    soma_quadrados += valor_lido * valor_lido;
    delay(2);
  }

  // Calcula tensão RMS com fator de calibração
  float media_quadrados = soma_quadrados / numero_amostras;
  float tensao_sensor_rms = sqrt(media_quadrados);
  float tensao_rms = (tensao_sensor_rms * fator_calibracao) / sqrt(2);
  
  Serial.print("Tensão RMS: ");
  Serial.print(tensao_rms);
  Serial.println(" V");
  
  client.publish("tensao", String(tensao_rms).c_str()); // Envia por MQTT
}

// === LOOP PRINCIPAL ===
void loop()
{
  medirCorrenteAC();  // Lê corrente
  medirTensao();      // Lê tensão

  unsigned long currentMillis = millis();

  // Reconecta MQTT se necessário
  if (!client.connected())
  {
    reconnect();
  }

  // Leitura contínua do GPS
  while (gpsSerial.available() > 0) 
  {
    gps.encode(gpsSerial.read());

    // Quando nova localização estiver disponível
    if (gps.location.isUpdated()) 
    {
      Serial.println("--------------------------");
      Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6);
      Serial.println("--------------------------");
    }
  }

  // Mantém a conexão com o broker
  if (!client.loop())
  {
    client.connect("ESP32");
  }

  // Controle baseado em tempo (a cada 3s)
  if (currentMillis - previousMillis >= 3000)
  {
    previousMillis = currentMillis;
  }
}
