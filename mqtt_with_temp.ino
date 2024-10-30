#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Configuración del BME280
Adafruit_BME280 bme;
#define SEALEVELPRESSURE_HPA (1013.25)

// Configuración del WiFi
const char* ssid = "WifiJJok";
const char* password = "jjcampis2024";

// Configuración del MQTT
const char* mqttServer = "hidroponia.local";
const int mqttPort = 1883;
const char* mqttUser = ""; // Usuario si es necesario
const char* mqttPassword = ""; // Contraseña si es necesario

WiFiClient espClient;
PubSubClient client(espClient);

// Configuración del sensor de pH
const int pHSensorPin = A0;
const int numReadings = 50;
float readings[numReadings];

// Configuración del DS18B20
#define ONE_WIRE_BUS 4 // Pin de datos del DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float temperatureDS18B20 = 0.0;

// Declaración de funciones
void setupBME280();
void setupPH();
float getPH();
void sendData();
void reconnect();

void setup() {
  Serial.begin(115200);
  setupBME280();
  setupPH();
  sensors.begin(); // Inicializar el DS18B20
  
  // Conectar a WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conectado a WiFi");

  // Configurar MQTT
  client.setServer(mqttServer, mqttPort);

  reconnect();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  sendData();
  delay(5000);
}

void setupBME280() {
  if (!bme.begin(0x76)) {
    Serial.println("No se pudo encontrar un sensor BME280 válido, revisa tu conexión!");
    while (1);
  }
}

void setupPH() {
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
}

float getPH() {
  float total = 0;
  for (int i = 0; i < numReadings; i++) {
    readings[i] = analogRead(pHSensorPin) * 3.3 / 4095.0;
    total += readings[i];
    delay(20);
  }
  float average = total / numReadings;
  float pH = 7.0 - ((average - 2.5) / 0.1841);
  return pH;
}

void getTemperatureDS18B20() {
  sensors.requestTemperatures();
  temperatureDS18B20 = sensors.getTempCByIndex(0);
}

void sendData() {
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure() / 100.0F;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float pH = getPH();
  getTemperatureDS18B20(); // Leer temperatura del DS18B20

  String jsonPayload = "{\"temperature\":" + String(temperature) +
                       ",\"humidity\":" + String(humidity) +
                       ",\"pressure\":" + String(pressure) +
                       ",\"altitude\":" + String(altitude) +
                       ",\"pH\":" + String(pH) +
                       ",\"water_temp\":" + String(temperatureDS18B20) + "}";

  Serial.println("Enviando datos a MQTT: ");
  Serial.println(jsonPayload);

  client.publish("sensor/data", jsonPayload.c_str());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Intentando conectar al broker MQTT...");
    // Attempt to connect
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("conectado");
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" intentar de nuevo en 5 segundos");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
