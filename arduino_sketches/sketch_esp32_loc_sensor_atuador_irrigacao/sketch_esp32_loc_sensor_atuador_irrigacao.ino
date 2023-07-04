// Programação do sensor físico de umidade do solo e do atuador (bomba de irrigação de água) para o sistema Planto IoT

#include <WiFi.h>
#include <PubSubClient.h>
#include "WiFiCredentials.h"
#include "MQTTCredentials.h"

// WiFi
WiFiClient wifiClient;

// MQTT Server
#define ID_MQTT "ESP32NODEMCU2"
#define TOPIC_PUBLISH "sos_acionado"
PubSubClient MQTT(wifiClient);

// Task handles
TaskHandle_t mqttTaskHandle;

// Button and LED pins
const int leituraPinoDigitalBotaoSOS = 22;
const int pinoDigitalAcionamentoLED = 23;

// Variables
bool sosAcionado = false;
int initMillis = 0;
int millisAcionamento = 0;
bool sosAtivo = false;

// Function declarations
void mqttTask(void* parameter);
void buttonTask(void* parameter);
void subscribeSOSTopicMQTT();
void callbackLidarComMensagensMQTT(char* topic, byte* message, unsigned int length);
void lerBotaoSOS();
void conectarWiFi();
void conectarMQTT();

void setup() {
  Serial.begin(115200);

  conectarWiFi();
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);

  // Definir pin modes
  pinMode(pinoDigitalAcionamentoLED, OUTPUT);
  pinMode(leituraPinoDigitalBotaoSOS, INPUT_PULLUP);

  initMillis = millis();

  // Create MQTT task - FreeRTOS
  xTaskCreatePinnedToCore(mqttTask, "mqttTask", 4096, NULL, 1, &mqttTaskHandle, 0);

  // Create button task - FreeRTOS
  xTaskCreatePinnedToCore(buttonTask, "buttonTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // Empty. Don't put any blocking code in the main loop.
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

  Serial.print("Conectando-se na rede: ");
  Serial.print(SSID);
  Serial.println("  Aguarde!");

  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Conectado com sucesso, na rede: ");
  Serial.print(SSID);
  Serial.print("  IP obtido: ");
  Serial.println(WiFi.localIP());
}

void conectarMQTT() {
  while (!MQTT.connected()) {
    Serial.print("Conectando ao Broker MQTT: ");
    Serial.println(BROKER_MQTT);

    if (MQTT.connect(ID_MQTT)) {
      Serial.println("Conectado ao Broker com sucesso!");
      subscribeSOSTopicMQTT();
    } else {
      Serial.println("Não foi possivel se conectar ao broker.");
      Serial.println("Nova tentativa de conexao em 10s");
      delay(10000);
    }
  }
}

void subscribeSOSTopicMQTT() {
  MQTT.subscribe("sos_acionado");
  MQTT.setCallback(callbackLidarComMensagensMQTT);
}

void callbackLidarComMensagensMQTT(char* topic, byte* message, unsigned int length) {
  Serial.print("Mensagem chegou via broker MQTT [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.write(message, length);
  Serial.println();

  if (strstr((char*)message, "SOS TRUE") != NULL) {
    sosAtivo = true;
  } else if (strstr((char*)message, "SOS FALSE") != NULL) {
    sosAtivo = false;
  }

  digitalWrite(pinoDigitalAcionamentoLED, sosAtivo);
}

// Tarefa de ler os inputs do botão
void buttonTask(void* parameter) {
  while (true) {
    lerBotaoSOS();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void lerBotaoSOS() {
  sosAcionado = digitalRead(leituraPinoDigitalBotaoSOS) == LOW;

  if (sosAcionado && (millis() > millisAcionamento + 2000)) {
    MQTT.publish(TOPIC_PUBLISH, "SOS TRUE");
    Serial.println("Botão acionado - SOS TRUE");

    digitalWrite(pinoDigitalAcionamentoLED, HIGH);
    millisAcionamento = millis();
  }

  if (millis() == millisAcionamento + 2000) {
    digitalWrite(pinoDigitalAcionamentoLED, LOW);
  }
}