#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP280.h>
#include <LoRa.h>
#include <math.h>

#define LORA_SS 18
#define LORA_RST 14
#define LORA_DI0 26

Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);
Adafruit_BMP280 bmp;

QueueHandle_t xQueueAccelData;
QueueHandle_t xQueueAcXAxisData;
QueueHandle_t xQueueAcYAxisData;
QueueHandle_t xQueueAcZAxisData;
QueueHandle_t xQueueTempData;

void initSensors() {
  if (!adxl.begin(0x53)) {
    Serial.println("Erro ao iniciar ADXL345");
    return;
  }

  if(!bmp.begin(0x76)) {
    Serial.println("Erro ao iniciar BMP280");
    return;
  }
}

void readADXL345(void *pvParameters) {
  while (1) {
    double xAxis = adxl.getX();
    double yAxis = adxl.getY();
    double zAxis = adxl.getZ();
    double accelMovement = sqrt((adxl.getX() * adxl.getX())  + (adxl.getY() * adxl.getY()) + (adxl.getZ() * adxl.getZ()));

    if(xQueueSend(xQueueAccelData, &accelMovement, portMAX_DELAY) != pdPASS) {
      Serial.println("Erro ao enviar valores do sensor para a fila.");
    }
    if(xQueueSend(xQueueAcXAxisData, &xAxis, portMAX_DELAY) != pdPASS) {
      Serial.println("Erro ao enviar valores do eixo x para a fila.");
    }
    if(xQueueSend(xQueueAcYAxisData, &yAxis, portMAX_DELAY) != pdPASS) {
      Serial.println("Erro ao enviar valores do eixo y para a fila.");
    }
    if(xQueueSend(xQueueAcZAxisData, &zAxis, portMAX_DELAY) != pdPASS) {
      Serial.println("Erro ao enviar valores do eixo z para a fila.");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void readBMP280(void *pvParameters) {
  while(1) {
    double readTemperature = bmp.readTemperature();

    if(xQueueSend(xQueueTempData, &readTemperature, portMAX_DELAY) != pdPASS) {
      Serial.println("Erro ao enviar valores de temperatura para a fila.");
    }

    LoRa.beginPacket();
    LoRa.print(readTemperature);
    LoRa.endPacket();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void sendLora(void *pvParameters) {
  String accelAxisData;
  double temperatureData, accelData, accelXData, accelYData, accelZData;
  while(1) {

    xQueueReceive(xQueueAccelData, &accelData, portMAX_DELAY);
    xQueueReceive(xQueueTempData, &temperatureData, portMAX_DELAY);
    xQueueReceive(xQueueAcXAxisData, &accelXData, portMAX_DELAY);
    xQueueReceive(xQueueAcYAxisData, &accelYData, portMAX_DELAY);
    xQueueReceive(xQueueAcZAxisData, &accelZData, portMAX_DELAY);

    String loraPayload =
            "{\"temperatura\":" + String(temperatureData) +
            ",\"accel\":" + String(accelData) +
            ",\"axisX\":" + String(accelXData) +
            ",\"axisY\":" + String(accelYData) +
            ",\"axisZ\":" + String(accelZData) +
            "}";

    Serial.println("");
    Serial.println("LoRa Packet: ");
    Serial.print(loraPayload);

    LoRa.beginPacket();
    LoRa.print(loraPayload);
    LoRa.endPacket();

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void I2CScanner() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000); // Esperar 5 segundos antes de escanear novamente.
}

void setup() {
  Serial.begin(115200);

  xQueueAccelData = xQueueCreate(5, sizeof(double));
  xQueueTempData = xQueueCreate(5, sizeof(double));
  xQueueAcXAxisData = xQueueCreate(5, sizeof(double));
  xQueueAcYAxisData = xQueueCreate(5, sizeof(double));
  xQueueAcZAxisData = xQueueCreate(5, sizeof(double));
  if (xQueueAccelData == NULL || xQueueTempData == NULL || xQueueAcXAxisData == NULL || xQueueAcYAxisData == NULL || xQueueAcZAxisData == NULL) {
    Serial.print("Erro ao criar a fila.");
    while (1);
  }

  initSensors();
  //I2CScanner();

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DI0);
  if (!LoRa.begin(915E6)) {
    Serial.println("Falha ao inicializar LoRa!");
    while (1);
  }
  Serial.println("LoRa inicializado");

  xTaskCreate(readADXL345, "Task_ADXL345", 2048, NULL, 2, NULL);
  xTaskCreate(readBMP280, "Task_BMP280", 2048, NULL, 1, NULL);
  xTaskCreate(sendLora, "Task_sendLora", 2048, NULL, 3, NULL);
}

void loop(){}