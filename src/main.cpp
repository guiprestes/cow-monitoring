#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP280.h>
#include <LoRa.h>

#define LORA_SS 18
#define LORA_RST 14
#define LORA_DI0 26
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27

Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);
Adafruit_BMP280 bmp;

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
  sensors_event_t event;
  while (1) {
    adxl.getEvent(&event);
    Serial.print("Acelerometro X: "); Serial.println(event.acceleration.x);
    Serial.print("Acelerometro Y: "); Serial.println(event.acceleration.y);
    Serial.print("Acelerometro Z: "); Serial.println(event.acceleration.z);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void readBMP280(void *pvParameters) {
  while(1) {
    float temperature = bmp.readTemperature();
    Serial.print("Temperatura: ");
    Serial.println(temperature);
    Serial.println(" ºC");
    vTaskDelay(60000 / portTICK_PERIOD_MS);
  }
}

void sendData(void *pvParameters) {
  while(1) {
    String sensorData = "X: " + String(adxl.getX()) + " Y:" + String(adxl.getY()) + " Z:" + String(adxl.getZ()) +
            " Temperatura: " + String(bmp.readTemperature());
    LoRa.beginPacket();
    LoRa.print(sensorData);
    LoRa.endPacket();
    Serial.println("Dados Enviados!");
    vTaskDelay(5000/portTICK_PERIOD_MS);
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

  initSensors();
  //I2CScanner();

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DI0);
  if (!LoRa.begin(915E6)) {
    Serial.println("Falha ao inicializar LoRa!");
    while (1);
  }
  Serial.println("LoRa inicializado");

  xTaskCreate(readADXL345, "Task_ADXL345", 4096, NULL, 1, NULL);
  xTaskCreate(readBMP280, "Task_BMP280", 2048, NULL, 1, NULL);
  xTaskCreate(sendData, "Task_LoRa", 2048, NULL, 2, NULL);
}

void loop(){}