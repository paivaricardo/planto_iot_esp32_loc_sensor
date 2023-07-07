// Planto IoT - Sensor e Atuador físico com ESP32 (DOIT ESP32 DEVKIT V.1)
// Programação do sensor físico de umidade do solo e do atuador (bomba de irrigação de água) para o sistema Planto IoT

#include <Preferences.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include "WiFiCredentials.h"
#include "MQTTCredentials.h"
#include <TimeLib.h>
#include "time.h"

// WiFi
WiFiClient wifiClient;

// MQTT Server
#define ID_MQTT "ESP32PLANTOIOTLOCALSENAT00001"
PubSubClient MQTT(wifiClient);

// UUIDs
#define UUID_SENSOR_00001 "fa00001b-1cb5-442b-87d5-654b7d2c1e9a"
#define UUID_ATUADOR_00001 "fb000012-ca1c-4c56-9294-1fcd77213390"

// Dados para obter o tempo local atualizado
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long gmtOffset_sec = -10800;
const int daylightOffset_sec = 0;

// Task handles
TaskHandle_t atuadorMQTTTaskHandle;
TaskHandle_t sensorReadingsTaskHandle;

// Tópicos MQTT
std::string topicoMQTTReceberComandoAtuadorString = "planto-iot-sensores/atuadores/" + std::string(UUID_ATUADOR_00001) + "/A";
const char* topicoMQTTReceberComandoAtuador = topicoMQTTReceberComandoAtuadorString.c_str();
const char* topicoMQTTEnviarLeituraSensor = "planto-iot-sensores/planto-iot-fazenda/1/S";

// Pinos digitais e analógicos para leitura dos sensores
const int leituraPinoAnalogicoSensorUmidadeSolo = 34;
const int pinoDigitalAcionamentoLEDLeituraUmidadeSolo = 32;

// Pinos digitais para acionamento dos atuadores
const int pinoDigitalAcionamentoLEDAtuador = 22;
const int pinoDigitalAcionamentoBuzzerAtuador = 23;
const int pinoDigitalAcionamentoReleBombaAgua = 21;

// Variables
int initMillis = 0;
int millisAcionamento = 0;

// Function declarations
void atuadorMQTTTask(void* parameter);
void sensorReadingsTask(void* parameter);
void subscribeAcionamentoAtuadorTopicMQTT();
void callbackLidarComMensagensMQTTAtuador(char* topic, byte* message, unsigned int length);
void conectarWiFi();
void conectarMQTT();

void setup() {
  Serial.begin(115200);

  // Inicialização dos pinos
  // Inicializar o led interno da placa ESP32
  pinMode(LED_BUILTIN, OUTPUT);

  // Inicialização dos pinos do sensor
  pinMode(leituraPinoAnalogicoSensorUmidadeSolo, INPUT);
  pinMode(pinoDigitalAcionamentoLEDLeituraUmidadeSolo, OUTPUT);

  // Inicialização pinos do atuador
  pinMode(pinoDigitalAcionamentoLEDAtuador, OUTPUT);
  pinMode(pinoDigitalAcionamentoBuzzerAtuador, OUTPUT);
  pinMode(pinoDigitalAcionamentoReleBombaAgua, OUTPUT);

  digitalWrite(pinoDigitalAcionamentoReleBombaAgua, HIGH);

  // Conectar o WiFi
  conectarWiFi();

  // Definir o servidor MQTT - Host e porta
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);

  // Definir o tempo inicial do dispositivo
  initMillis = millis();

  // Create MQTT tasks - FreeRTOS
  xTaskCreatePinnedToCore(atuadorMQTTTask, "atuadorMQTTTask", 4096, NULL, 1, &atuadorMQTTTaskHandle, 0);
  xTaskCreatePinnedToCore(sensorReadingsTask, "sensorReadingsTask", 4096, NULL, 1, &sensorReadingsTaskHandle, 1);
}

void loop() {
  // Empty. Don't put any blocking code in the main loop.
}

void blinkLed(int ledPin, int times, int intervalDurationMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(ledPin, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(intervalDurationMs);
    digitalWrite(ledPin, LOW);
    delay(intervalDurationMs);  // turn the LED on (HIGH is the voltage level)
  }
}

void atuadorMQTTTask(void* parameter) {
  while (true) {
    if (!MQTT.connected()) {
      conectarMQTT();
    }

    MQTT.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void conectarWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  Serial.print("Conectando-se à rede: ");
  Serial.print(SSID);
  Serial.println("  Aguarde!");

  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }

  // Obter a data e a hora locais
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
  Serial.println("Obtidas datas e horas atualizadas do servidor NTP.");


  Serial.println();
  Serial.print("Conectado com sucesso, na rede: ");
  Serial.print(SSID);
  Serial.print("  IP obtido: ");
  Serial.println(WiFi.localIP());

  // Na conexão com sucesso ao Wi-Fi, vai piscar 3 vezes devagar.
  blinkLed(LED_BUILTIN, 3, 1000);

}

void conectarMQTT() {
  while (!MQTT.connected()) {
    Serial.print("Conectando ao Broker MQTT: ");
    Serial.println(BROKER_MQTT);

    if (MQTT.connect(ID_MQTT)) {
      Serial.println("Conectado ao Broker com sucesso!");

      // Na conexão com sucesso ao MQTT Server, vai piscar 5 vezes rápido.
      blinkLed(LED_BUILTIN, 5, 200);
      subscribeAcionamentoAtuadorTopicMQTT();
    } else {
      Serial.println("Não foi possivel se conectar ao broker.");
      Serial.println("Nova tentativa de conexao em 10s");
      delay(10000);
    }
  }
}

void subscribeAcionamentoAtuadorTopicMQTT() {
  MQTT.subscribe(topicoMQTTReceberComandoAtuador);
  MQTT.setCallback(callbackLidarComMensagensMQTTAtuador);
}

void acionarBuzzerIntermitente(int frequency, int times, int duration) {
  for (int i = 0; i < times; i++) {
    tone(pinoDigitalAcionamentoBuzzerAtuador, frequency);
    delay(duration);
    noTone(pinoDigitalAcionamentoBuzzerAtuador);
    delay(duration);
  }
}

void acionarBuzzerContinuo(int frequency, int duration) {
  tone(pinoDigitalAcionamentoBuzzerAtuador, frequency, duration);
}

void acionarReleBombaAgua(int duration) {
  digitalWrite(pinoDigitalAcionamentoReleBombaAgua, LOW);
  delay(duration);
  digitalWrite(pinoDigitalAcionamentoReleBombaAgua, HIGH);
}

void acionarAtuadorBombaIrrigacao(int duration) {
  const int buzzerFrequencyBomba = 440;

  acionarReleBombaAgua(duration);
  acionarBuzzerContinuo(buzzerFrequencyBomba, duration);
  blinkLed(pinoDigitalAcionamentoLEDAtuador, 1, duration);
  acionarBuzzerIntermitente(buzzerFrequencyBomba, 3, 200);
}

void callbackLidarComMensagensMQTTAtuador(char* topic, byte* message, unsigned int length) {
  // Fazer o print da mensagem do atuador via console serial
  Serial.print("[ATUADOR - INFO] Mensagem chegou via broker MQTT [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.write(message, length);
  Serial.println();
  blinkLed(LED_BUILTIN, 2, 200);

  // Converter os bytes da mensagem recebida em char*
  char* inputJson = reinterpret_cast<char*>(message);

  // Desserializar JSON recebido pela mensagem com biblioteca
  // char* input;
  // size_t inputLength; (optional)

  StaticJsonDocument<64> doc;

  DeserializationError error = deserializeJson(doc, inputJson, length);

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  const char* uuidSensorAtuador = doc["uuidSensorAtuador"];  // "fb000012-ca1c-4c56-9294-1fcd77213390"
  const char* dataHoraAcionamentoLeitura = doc["dataHoraAcionamentoLeitura"];
  long tipoSinal = doc["tipoSinal"];                 // 50000
  int quantidadeAtuacao = doc["quantidadeAtuacao"];  // 500

  // Se o sinal for 50000, que é o sinal para acionamento do atuador, acionar a bomba de irrigação
  if (tipoSinal == 50000) {
    // Com os parâmetros recebidos da mensagem, acionar a bomba de irrigação
    acionarAtuadorBombaIrrigacao(quantidadeAtuacao);
  }
}

void sensorReadingsTask(void* parameter) {
  while (true) {
    // Read the sensor value using analogRead()
    int sensorValue = analogRead(leituraPinoAnalogicoSensorUmidadeSolo);

    time_t now = time(nullptr);

    // Convert the sensor value to a percentage
    int percentualUmidadeSolo = map(sensorValue, 0, 4095, 100, 0);

    // formatar uma string com a data e a hora atuais
    char datetimeString[40];
    sprintf(datetimeString, "%04d-%02d-%02dT%02d:%02d:%02d.%03lu-00:00",
            year(now), month(now), day(now),
            hour(now), minute(now), second(now), millis() % 1000);

    // Create a JSON document to hold the sensor reading data
    StaticJsonDocument<128> doc;

    // Populate the JSON document with the sensor reading data
    doc["uuidSensorAtuador"] = UUID_SENSOR_00001;
    doc["dataHoraAcionamentoLeitura"] = datetimeString;
    doc["tipoSinal"] = 10000;
    doc["informacoesEspecificasSensor"]["percentualUmidadeSolo"] = percentualUmidadeSolo;

    // Serialize the JSON document to a string
    String jsonString;
    serializeJson(doc, jsonString);

    // Convert the string to a const char* for MQTT publishing
    const char* message = jsonString.c_str();

    // Publish the sensor reading to the MQTT broker
    MQTT.publish(topicoMQTTEnviarLeituraSensor, message);

    // Acionar o led verde para indicar envio da leitura
    blinkLed(pinoDigitalAcionamentoLEDLeituraUmidadeSolo, 1, 1000);

    // Logar a mensagem publicada via MQTT no console
    // Print MQTT message to serial console
    Serial.print("[SENSOR - INFO] Publicada mensagem de leitura do sensor no tópico ");
    Serial.print(topicoMQTTEnviarLeituraSensor);
    Serial.print(", com o seguinte conteúdo: ");
    Serial.println(message);

    // Delay before the next reading
    vTaskDelay(7000 / portTICK_PERIOD_MS);
  }
}
