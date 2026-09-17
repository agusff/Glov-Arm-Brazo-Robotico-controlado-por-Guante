/*
 * Glov-Arm — Firmware del GUANTE (nodo transmisor ESP32-C3)
 * ---------------------------------------------------------------
 *
 * Configuración necesaria en Arduino IDE (Tools):
 *   - Board: "ESP32C3 Dev Module" (paquete esp32 by Espressif Systems, 3.3.11+)
 *   - USB CDC On Boot: "Enabled"  <- IMPRESCINDIBLE en placas Super Mini
 *     (USB nativo sin puente UART): sin esto, Serial.print() no sale
 *     por el puerto USB. Solo hacer si se quiere depurar.
 *   - Librería: "MPU6050" de Electronic Cats (Sketch > Include Library >
 *     Manage Libraries), da I2Cdev.h y MPU6050.h.
 */

#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "data_packet.h"

// Pines I2C ESP-C3
#define SDA 8
#define SLC 9

// Canal WiFi fijo para ESP-NOW
#define ESPNOW_WIFI_CHANNEL 1


// MAC DEL RECEPTOR (ESP_brazo) 
uint8_t broadcastAddress[] = { 0x1C, 0xDB, 0xD4, 0xC6, 0x76, 0x38 };

// ESTRUCTURA DE DATOS PARA ESP-NOW
GloveDataPacket_t datosGuante;
esp_now_peer_info_t peerInfo;


// Estados de la maquina
enum EstadoGuante { GUANTE_IDLE,
                    GUANTE_ACTIVO };
volatile EstadoGuante estadoGuante = GUANTE_IDLE;


// Sensores Hall 
const int pinHallIndice = 0;   // ADC1_CH0
const int pinHallCorazon = 1;  // ADC1_CH1
const int UMBRAL_HALL = 3000;
// Filtro EMA 
float ema_indice = 0;
float ema_corazon = 0;
const float ALPHA_FLEX = 0.15;
// Rango Analogo-Mecanico
const int MIN_HALL = 1800;  // Valor ADC con maxima intensidad de campo mag.
const int MAX_HALL = 2600;  // Valor ADC con minima intensidad de campo mag.

// Giroscopio
int16_t ax, ay, az, gx, gy, gz;
unsigned long tiempo_prev;
float dt;
float angulo_x = 0, angulo_y = 0;
float error_gx = 0, error_gy = 0;

MPU6050 sensor;


unsigned long tiempoUltimaLectura = 0;
const int INTERVALO_LECTURA = 10;  // 100 Hz (transmisión fluida) — solo corre en estado ACTIVO


#if ESP_ARDUINO_VERSION_MAJOR >= 3

// Depuracion
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // Se puede usar para chequear pérdidas de paquetes si fuese necesario.
}

void OnCtrlRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  Serial.printf("\n>> INT. RX DISPARADA | Bytes recibidos: %d\n", len);
  if (len != sizeof(CtrlMessage_t)) {
    Serial.println("-> ERROR: Tamaño de paquete incorrecto. Descartado.");
    return;
  }
  CtrlMessage_t msg;
  memcpy(&msg, incomingData, sizeof(msg));
  Serial.printf("-> Comando recibido: 0x%02X\n", msg.cmd);

  if (msg.cmd == CTRL_CMD_WAKE) {
    estadoGuante = GUANTE_ACTIVO;
    tiempoUltimaLectura = millis(); 
    Serial.println("-> SISTEMA DESPIERTO. Iniciando biometría...");
  } else if (msg.cmd == CTRL_CMD_SLEEP) {
    estadoGuante = GUANTE_IDLE;
    Serial.println("-> SISTEMA DORMIDO. Ahorro de energía activado.");
  }
}

#else

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

void OnCtrlRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  Serial.printf("\n>> INT. RX DISPARADA | Bytes recibidos: %d\n", len);
  if (len != sizeof(CtrlMessage_t)) {
    Serial.println("-> ERROR: Tamaño de paquete incorrecto. Descartado.");
    return;
  }
  CtrlMessage_t msg;
  memcpy(&msg, incomingData, sizeof(msg));
  Serial.printf("-> Comando recibido: 0x%02X\n", msg.cmd);

  if (msg.cmd == CTRL_CMD_WAKE) {
    estadoGuante = GUANTE_ACTIVO;
    tiempoUltimaLectura = millis();
    Serial.println("-> SISTEMA DESPIERTO. Iniciando biometría...");
  } else if (msg.cmd == CTRL_CMD_SLEEP) {
    estadoGuante = GUANTE_IDLE;
    Serial.println("-> SISTEMA DORMIDO. Ahorro de energía activado.");
  }
}

#endif

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA, SCL);

  //Incializacion del giroscopio
  sensor.initialize();
  if (!sensor.testConnection()) {
    Serial.println("Error al iniciar MPU6050.");
  }

  // --- Calibración giroscopio (offset estático, sensor en reposo) ---
  delay(1000);
  for (int i = 0; i < 200; i++) {
    sensor.getRotation(&gx, &gy, &gz);
    error_gx += gx;
    error_gy += gy;
    delay(10);
  }
  error_gx /= 200.0;
  error_gy /= 200.0;
  tiempo_prev = millis();

  // --- CONFIGURACIÓN WI-FI & ESP-NOW ---
  WiFi.mode(WIFI_STA);  // Modo Estación requerido para ESP-NOW
  esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnCtrlRecv);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error al añadir el receptor (Peer)");
    return;
  }

  Serial.println("Guante listo, en estado IDLE. Esperando CTRL_CMD_WAKE del receptor.");
}

void loop() {

  if (estadoGuante != GUANTE_ACTIVO) {
    return;
  }

  unsigned long tiempoActual = millis();

  // TAREA DE LECTURA, EMPAQUETADO Y ENVÍO (cada 10 ms)
  if (tiempoActual - tiempoUltimaLectura >= INTERVALO_LECTURA) {
    tiempoUltimaLectura = tiempoActual;

    // Procesamiento del MPU6050
    sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    dt = tiempoActual - tiempo_prev;
    tiempo_prev = tiempoActual;

    float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14159);
    float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14159);

    float girosc_tasa_x = (gx - error_gx) / 131.0;  // 131 LSB/(°/s) @ ±250°/s (datasheet MPU-6050)
    float girosc_tasa_y = (gy - error_gy) / 131.0;

    // Filtro complementario: 96% giróscopo (integración, deriva a
    // largo plazo) + 4% acelerómetro (referencia absoluta, ruido
    // vibratorio a corto plazo).
    angulo_x = 0.96 * (angulo_x + (girosc_tasa_x * (dt / 1000.0))) + 0.04 * accel_ang_x;
    angulo_y = 0.96 * (angulo_y + (girosc_tasa_y * (dt / 1000.0))) + 0.04 * accel_ang_y;

    // Procesamiento Sensores Hall 
    int lecturaIndice  = analogRead(pinHallIndice);
    int lecturaCorazon = analogRead(pinHallCorazon);

    // Filtro EMA
    ema_indice = (ALPHA_FLEX * lecturaIndice) + ((1.0 - ALPHA_FLEX) * ema_indice);
    ema_corazon = (ALPHA_FLEX * lecturaCorazon) + ((1.0 - ALPHA_FLEX) * ema_corazon);

    // Mapeo lineal ADC - Grados
    int anguloIndiceCalculado = map((int)ema_indice, MIN_HALL, MAX_HALL, 0, 180);
    int anguloCorazonCalculado = map((int)ema_corazon, MIN_HALL, MAX_HALL, 0, 180);

    // Acondicionamiento Inercial (MPU6050)
    // Suponiendo que el filtro arroja valores de -90 a 90 grados físicos, los centramos a 0-180
    int ang_x_final = (int)(-angulo_x + 90.0); // El signo menos es por la posicion fisica del mpu, se debe ajustar dependiendo la posicion
    int ang_y_final = (int)(angulo_y + 90.0);

    // Carga del Struct asegurando saturación de seguridad (0 a 180)
    datosGuante.angulo_x = (uint8_t)constrain(ang_x_final, 0, 180);
    datosGuante.angulo_y = (uint8_t)constrain(ang_y_final, 0, 180);
    datosGuante.pinzaIndice = (uint8_t)constrain(anguloIndiceCalculado, 0, 180);
    datosGuante.pinzaCorazon = (uint8_t)constrain(anguloCorazonCalculado, 0, 180);

    // Depuracion
    Serial.printf("Codo (X): %d° | Hombro (Y): %d° | Pinza: %d° | Muñeca: %d°\n",
                  datosGuante.angulo_x, datosGuante.angulo_y,
                  datosGuante.pinzaIndice, datosGuante.pinzaCorazon);

    // Transmision inalambrica
    esp_err_t resultado = esp_now_send(broadcastAddress,
                                       (uint8_t *)&datosGuante, sizeof(datosGuante));
    (void)resultado;  // ver OnDataSent() para diagnóstico asíncrono real
  }
}

