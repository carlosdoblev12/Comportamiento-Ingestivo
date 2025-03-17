/*
 * Proyecto: Monitoreo de GNSS, IMU, almacenamiento en SD y transmisión LoRa
 * Autor: Carlos Mario Minu Quiroga
 * Información de contacto: Universidad de los Andes - cm.minu10@uniandes.edu.co
 * Descripción: 
 *  Este código permite la adquisición de datos GNSS e IMU en un sistema embebido,
 *  almacenando los datos en una tarjeta SD utilizando el protocolo SDIO y enviándolos 
 *  a través de LoRa. También gestiona indicadores LED RGB para reflejar el estado del sistema.
 */

#include <Wire.h>  // Necesario para comunicación I2C con GNSS e IMU
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // Librería para módulo GNSS u-blox
#include "SparkFunLSM6DSO.h" // Librería para el sensor IMU LSM6DSO
#include "FS.h"
#include "SD_MMC.h" // Para la gestión del almacenamiento SD usando SDIO
#include <LoRa.h>   // Para la comunicación mediante LoRa
#include <SPI.h>
#include <FastLED.h> // Para controlar el LED RGB WS2812

// Definición de pines para la interfaz SDIO
int pin_sdioCLK = 38;  // Pin de reloj (CLK)
int pin_sdioCMD = 34;  // Pin de comando (CMD)
int pin_sdioD0 = 39;   // Línea de datos D0
int pin_sdioD1 = 40;   // Línea de datos D1
int pin_sdioD2 = 47;   // Línea de datos D2
int pin_sdioD3 = 33;   // Línea de datos D3

// Definición de pines para el módulo LoRa
#define SS_PIN 1      // Chip Select
#define RST_PIN 2     // Reset
#define DIO0_PIN 4    // Interrupción de datos recibidos

// Configuración del LED RGB WS2812
#define LED_PIN 2      // Pin de control del LED RGB
#define COLOR_ORDER GRB // Orden de colores del LED
#define CHIPSET WS2812  // Tipo de LED
#define NUM_LEDS 1      // Cantidad de LEDs conectados
#define BRIGHTNESS 25   // Brillo del LED

CRGB leds[NUM_LEDS]; // Array para controlar el LED RGB

bool ledState = false; // Estado del LED (para parpadeo)

// Objetos para GNSS e IMU
SFE_UBLOX_GNSS myGNSS;
LSM6DSO myIMU;

// Variables de tiempo para actualización de sensores
long lastGNSSUpdate = 0;
long lastIMUUpdate = 0;
const int IMUInterval = 500;  // Intervalo de actualización del IMU en ms
const int GNSSInterval = 1000; // Intervalo de actualización del GNSS en ms

// Variables para almacenar datos GNSS
unsigned long unixEpoch = 0;
long latitude = 0;
long longitude = 0;
long altitude = 0;

// Nombre del archivo en la SD
String fileName = "/data.csv";
File dataFile;

void setup() {
  Serial.begin(115200);
  if (Serial) {
    while (!Serial); // Espera si está conectado por USB
  }
  Serial.println("Inicializando sistema...");

  // Inicializar LED RGB
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  setLED(CRGB::Red); // LED en rojo indica inicio del sistema

  // Configuración de pines SDIO y verificación de tarjeta SD
  if (!SD_MMC.setPins(pin_sdioCLK, pin_sdioCMD, pin_sdioD0, pin_sdioD1, pin_sdioD2, pin_sdioD3)) {
    Serial.println("Error en la asignación de pines SDIO.");
    return;
  }

  if (!SD_MMC.begin()) {
    Serial.println("Error al montar la tarjeta SD.");
    setLED(CRGB::Blue); // Azul indica error en la SD
    return;
  }
  Serial.println("Tarjeta SD montada correctamente.");

  // Inicializar comunicación I2C
  Wire.begin();

  // Configurar GNSS
  if (!myGNSS.begin()) {
    Serial.println(F("No se detectó el módulo GNSS. Deteniendo ejecución."));
    setLED(CRGB::Purple);
    while (1);
  }
  myGNSS.setI2COutput(COM_TYPE_UBX);
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  Serial.println("GNSS inicializado.");

  // LED amarillo indica que el GNSS está buscando señal
  setLED(CRGB::Yellow);
  Serial.println("Esperando señal de GPS...");
  while (myGNSS.getFixType() < 2) {  // Espera hasta obtener al menos un fix 2D
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\n¡GPS Fix adquirido!");

  // Inicializar IMU
  if (!myIMU.begin()) {
    Serial.println("No se pudo conectar al IMU. Deteniendo ejecución.");
    setLED(CRGB::Pink);
    while (1);
  }
  if (myIMU.initialize(BASIC_SETTINGS)) {
    Serial.println("IMU configurado correctamente.");
  }

  // Inicializar LoRa
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(433E6)) {
    Serial.println("Error en la inicialización de LoRa.");
    setLED(CRGB::Chocolate);
    while (true);
  }
  LoRa.setSyncWord(0xA5);
  Serial.println("LoRa configurado correctamente.");

  // LED en verde indica que todo está funcionando
  setLED(CRGB::Green);
}

void loop() {
  // Actualización de GNSS
  if (millis() - lastGNSSUpdate > GNSSInterval) {
    lastGNSSUpdate = millis();

    if (myGNSS.getFixType() >= 2) {
      setLED(CRGB::Green); // LED verde indica GNSS activo
      unixEpoch = myGNSS.getUnixEpoch();
      latitude = myGNSS.getLatitude();
      longitude = myGNSS.getLongitude();
      altitude = myGNSS.getAltitude();
      Serial.println("Datos GNSS actualizados.");

      if (fileName == "/data.csv") { // Crear archivo con la primera marca de tiempo válida
        fileName = "/" + String(unixEpoch) + ".csv";
        Serial.print("Nuevo nombre de archivo: ");
        Serial.println(fileName);
        dataFile = SD_MMC.open(fileName, FILE_WRITE);
        if (dataFile) {
          Serial.println("Escribiendo encabezado en archivo.");
          dataFile.println("Unix Epoch,Latitude,Longitude,Altitude (mm),Accel X,Accel Y,Accel Z,Gyro X,Gyro Y,Gyro Z,Temp (F)");
          dataFile.close();
        } else {
          Serial.println("Error al crear archivo de datos.");
        }
      }
    } else {
      Serial.println("Esperando señal GPS válida...");
      setLED(CRGB::Yellow);
    }
  }

  // Actualización de IMU y almacenamiento de datos
  if (millis() - lastIMUUpdate > IMUInterval) {
    lastIMUUpdate = millis();
    float accelX = myIMU.readFloatAccelX();
    float accelY = myIMU.readFloatAccelY();
    float accelZ = myIMU.readFloatAccelZ();
    float gyroX = myIMU.readFloatGyroX();
    float gyroY = myIMU.readFloatGyroY();
    float gyroZ = myIMU.readFloatGyroZ();
    float tempF = myIMU.readTempF();

    Serial.println("Guardando datos en SD...");
    dataFile = SD_MMC.open(fileName, FILE_APPEND);
    if (dataFile) {
      String dataLine = String(unixEpoch) + "," + String(latitude) + "," + String(longitude) + "," +
                        String(altitude) + "," + String(accelX, 3) + "," + String(accelY, 3) + "," +
                        String(accelZ, 3) + "," + String(gyroX, 3) + "," + String(gyroY, 3) + "," +
                        String(gyroZ, 3) + "," + String(tempF, 3);
      dataFile.println(dataLine);
      dataFile.close();
      Serial.println("Datos guardados correctamente.");

      // Envío de datos por LoRa
      LoRa.beginPacket();
      LoRa.print(dataLine);
      LoRa.endPacket();
    } else {
      Serial.println("Error al abrir archivo para escritura.");
    }
  }
}

// Función para cambiar color del LED RGB
void setLED(CRGB color) {
  leds[0] = color;
  FastLED.show();
}
