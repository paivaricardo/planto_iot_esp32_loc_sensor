// Planto IoT - Sensores de luminosidade, temperatura e umidade do ar com ESP32 (DOIT ESP32 DEVKIT V.1)
// Programação do sensores físicos de luminosidade, temperatura e umidade do ar para o sistema Planto IoT

#include <Preferences.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <DHT.h>
#include <DHT_U.h>
#include "WiFiCredentials.h"
#include "MQTTCredentials.h"
#include <TimeLib.h>
#include "time.h"

// WiFi
WiFiClient wifiClient;

// MQTT Server
#define ID_MQTT "ESP32PLANTOIOTLOCALSENS00002"
PubSubClient MQTT(wifiClient);

// UUIDs
#define UUID_SENSOR_LUMIN_00002 "fa00002a-9244-47d5-8394-1b555f7733df"
#define UUID_SENSOR_UMIDADE_AR_00003 "fa00003a-37fb-4c5d-9b19-2468ff26915e"
#define UUID_SENSOR_TEMPERATURA_00004 "fa00004a-e3d3-49d6-9ed7-7906b2d40144"

// Definição do pino e do tipo do sensor DHT11
#define DHT_SENSOR_PIN 33
#define DHT_SENSOR_TYPE DHT11
DHT dht(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

// Dados para obter o tempo local atualizado
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long gmtOffset_sec = -10800;
const int daylightOffset_sec = 0;

// Task handles
TaskHandle_t mqttTaskHandle;
TaskHandle_t sensorLuminMQTTTaskHandle;
TaskHandle_t sensorTempUmidArMQTTTaskHandle;

// Tópicos MQTT
const char* topicoMQTTEnviarLeituraSensores = "planto-iot-sensores/planto-iot-fazenda/1/S";

// Pinos digitais e analógicos para leitura dos sensores e acionamento de leds
// Sensor de luminosidade
const int leituraPinoAnalogicoSensorLumin = 32;
const int pinoDigitalAcionamentoLEDLumin = 22;

// Sensor de temperatura e umidade do ar (DHT11)
const int pinoDigitalAcionamentoLEDDHT11 = 23;

// Variables
int initMillis = 0;

// Function declarations
// FreeRTOS Tasks
void mqttTask(void* parameter);
void sensorLuminMQTTTask(void* parameter);
void sensorTempUmidArMQTTTask(void* parameter);

// Other functions
void subscribeMensagensMQTT();
void callbackLidarComMensagensMQTT(char* topic, byte* message, unsigned int length);
void conectarWiFi();
void conectarMQTT();


void setup() {
  Serial.begin(115200);

  // Inicialização dos pinos
  // Inicializar o led interno da placa ESP32
  pinMode(LED_BUILTIN, OUTPUT);

  // Inicialização do sensor DHT
  dht.begin();
  pinMode(pinoDigitalAcionamentoLEDDHT11, OUTPUT);

  // Inicialização dos pinos do sensor de luminosidade
  pinMode(leituraPinoAnalogicoSensorLumin, INPUT);
  pinMode(pinoDigitalAcionamentoLEDLumin, OUTPUT);

  // Conectar ao WiFi
  conectarWiFi();

  // Definir servidor MQTT
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);

  // Definir o tempo inicial do dispositivo
  initMillis = millis();

  // Criar tarefas para execução concorrente por meio do sistem FreeRTOS
  xTaskCreatePinnedToCore(mqttTask, "mqttTask", 4096, NULL, 1, &mqttTaskHandle, 0);
  xTaskCreatePinnedToCore(sensorLuminMQTTTask, "sensorLuminMQTTTask", 4096, NULL, 1, &sensorLuminMQTTTaskHandle, 0);
  xTaskCreatePinnedToCore(sensorTempUmidArMQTTTask, "sensorTempUmidArMQTTTask", 4096, NULL, 1, &sensorTempUmidArMQTTTaskHandle, 1);
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

void mqttTask(void* parameter) {
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
  Serial.print("Conectado com sucesso à rede ");
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
      Serial.println("Conectado ao Broker MQTT com sucesso!");

      // Na conexão com sucesso ao MQTT Server, vai piscar 5 vezes rápido.
      blinkLed(LED_BUILTIN, 5, 200);
      subscribeMensagensMQTT();
    } else {
      Serial.println("Não foi possivel se conectar ao broker MQTT.");
      Serial.println("Nova tentativa de conexao em 10s");
      delay(10000);
    }
  }
}

void subscribeMensagensMQTT() {
  // MQTT.subscribe(topicoMQTTReceberComandoAtuador);
  // MQTT.setCallback(callbackLidarComMensagensMQTT);
}

void callbackLidarComMensagensMQTT(char* topic, byte* message, unsigned int length) {
  // Fazer o print da mensagem do atuador via console serial
  Serial.print("[SENSORES - INFO] Mensagem chegou via broker MQTT [");
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
}

void sensorLuminMQTTTask(void* parameter) {
  while (true) {
    // Resistência pull-down do sensor de luminosidade LDR
    float R1 = 10000.0;

    // Read the sensor value using analogRead()
    int luminSensorValue = analogRead(leituraPinoAnalogicoSensorLumin);
    float vin = 3.3;
    float analogRange = 4095.0;
    float vout = (luminSensorValue * (vin / analogRange));  // [Vout = ADC * (Vin / analogRange)] 4095 -> analog range
    float RLDR = (R1 * (vin - vout)) / vout;                // equação: [R-LDR =(R1 (Vin - Vout))/ Vout]            // Take the logarithm of RLDR (base 10)
    float indiceLuminosidadeLux = (5000000.0 / RLDR);

    // Obter a hora atual
    time_t now = time(nullptr);

    // formatar uma string com a data e a hora atuais
    char datetimeString[40];
    sprintf(datetimeString, "%04d-%02d-%02dT%02d:%02d:%02d.%03lu-00:00",
            year(now), month(now), day(now),
            hour(now), minute(now), second(now), millis() % 1000);

    // Create a JSON document to hold the sensor reading data
    StaticJsonDocument<128> doc;

    // Populate the JSON document with the sensor reading data
    doc["uuidSensorAtuador"] = UUID_SENSOR_LUMIN_00002;
    doc["dataHoraAcionamentoLeitura"] = datetimeString;
    doc["tipoSinal"] = 10000;
    doc["informacoesEspecificasSensor"]["indiceLuminosidade"] = indiceLuminosidadeLux;

    // Serialize the JSON document to a string
    String jsonString;
    serializeJson(doc, jsonString);

    // Convert the string to a const char* for MQTT publishing
    const char* message = jsonString.c_str();

    // Publish the sensor reading to the MQTT broker
    MQTT.publish(topicoMQTTEnviarLeituraSensores, message);

    // Acionar o led amarelo para indicar envio da leitura do sensor de luminosidade
    blinkLed(pinoDigitalAcionamentoLEDLumin, 2, 1000);

    // Logar a mensagem publicada via MQTT no console
    // Print MQTT message to serial console
    Serial.print("[SENSOR - LUMIN - INFO] Publicada mensagem de leitura do sensor no tópico ");
    Serial.print(topicoMQTTEnviarLeituraSensores);
    Serial.print(", com o seguinte conteúdo: ");
    Serial.println(message);

    // Delay before the next reading
    vTaskDelay(7000 / portTICK_PERIOD_MS);
  }
}

void publicarMensagemMQTTSensorTemp(float temperatureMeasurement, float indiceCalorCelsius) {
  time_t now = time(nullptr);

  // formatar uma string com a data e a hora atuais
  char datetimeString[40];
  sprintf(datetimeString, "%04d-%02d-%02dT%02d:%02d:%02d.%03lu-00:00",
          year(now), month(now), day(now),
          hour(now), minute(now), second(now), millis() % 1000);

  // Publicar mensagem ao sensor de temperatura
  // Create a JSON document to hold the sensor reading data
  StaticJsonDocument<128> doc;

  // Populate the JSON document with the sensor reading data
  doc["uuidSensorAtuador"] = UUID_SENSOR_TEMPERATURA_00004;
  doc["dataHoraAcionamentoLeitura"] = datetimeString;
  doc["tipoSinal"] = 10000;
  doc["informacoesEspecificasSensor"]["temperaturaCelsius"] = temperatureMeasurement;
  // doc["informacoesEspecificasSensor"]["indiceCalorCelsius"] = indiceCalorCelsius;

  // Serialize the JSON document to a string
  String jsonString;
  serializeJson(doc, jsonString);

  // Convert the string to a const char* for MQTT publishing
  const char* messageTemperature = jsonString.c_str();

  // Publish the sensor reading to the MQTT broker
  MQTT.publish(topicoMQTTEnviarLeituraSensores, messageTemperature);

  // Acionar o led branco para indicar envio da leitura (Já é feito pela tarefa de mensuração da umidade)
  blinkLed(pinoDigitalAcionamentoLEDDHT11, 1, 1000);

  // Logar a mensagem publicada via MQTT no console
  // Print MQTT message to serial console
  Serial.print("[SENSOR - TEMP - INFO] Publicada mensagem de leitura do sensor no tópico ");
  Serial.print(topicoMQTTEnviarLeituraSensores);
  Serial.print(", com o seguinte conteúdo: ");
  Serial.println(messageTemperature);
}

void publicarMensagemMQTTSensorUmidAr(float humidityMeasurement) {
  // Obter a hora atual
  time_t now = time(nullptr);

  // formatar uma string com a data e a hora atuais
  char datetimeString[40];
  sprintf(datetimeString, "%04d-%02d-%02dT%02d:%02d:%02d.%03lu-00:00",
          year(now), month(now), day(now),
          hour(now), minute(now), second(now), millis() % 1000);


  // Publicar mensagem ao sensor de umidade do ar
  // Create a JSON document to hold the sensor reading data
  StaticJsonDocument<128> doc;

  // Populate the JSON document with the sensor reading data
  doc["uuidSensorAtuador"] = UUID_SENSOR_UMIDADE_AR_00003;
  doc["dataHoraAcionamentoLeitura"] = datetimeString;
  doc["tipoSinal"] = 10000;
  doc["informacoesEspecificasSensor"]["percentualUmidadeAr"] = humidityMeasurement;

  // Serialize the JSON document to a string
  String jsonString;
  serializeJson(doc, jsonString);

  // Convert the string to a const char* for MQTT publishing
  const char* message = jsonString.c_str();

  // Publish the sensor reading to the MQTT broker
  MQTT.publish(topicoMQTTEnviarLeituraSensores, message);

  // Acionar o led branco para indicar envio da leitura do sensor de umidade
  // blinkLed(pinoDigitalAcionamentoLEDDHT11, 1, 1000);

  // Logar a mensagem publicada via MQTT no console
  // Print MQTT message to serial console
  Serial.print("[SENSOR - UMID AR - INFO] Publicada mensagem de leitura do sensor no tópico ");
  Serial.print(topicoMQTTEnviarLeituraSensores);
  Serial.print(", com o seguinte conteúdo: ");
  Serial.println(message);
}

void sensorTempUmidArMQTTTask(void* parameter) {
  while (true) {
    // Read the sensor value using dht functions
    float temperatureMeasurement = dht.readTemperature();
    float humidityMeasurement = dht.readHumidity();

    // Se a leitura falhar, abortar esta chamada de função e sair cedo (para tentar novamente)
    if (isnan(temperatureMeasurement) || isnan(humidityMeasurement)) {
      Serial.println(F("Falha em obter a leitura de temperatura do sensor DHT!"));
      return;
    }

    // Computar o índice de calor em Celsius (isFahreheit = false)
    float indiceCalorCelsius = dht.computeHeatIndex(temperatureMeasurement, humidityMeasurement, false);

    publicarMensagemMQTTSensorTemp(temperatureMeasurement, indiceCalorCelsius);
    publicarMensagemMQTTSensorUmidAr(humidityMeasurement);

    // Delay before the next reading
    vTaskDelay(7000 / portTICK_PERIOD_MS);
  }
}