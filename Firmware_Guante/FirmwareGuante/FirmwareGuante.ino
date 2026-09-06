/*
 * Glov-Arm — Firmware del GUANTE (nodo transmisor ESP32-C3)
 * ---------------------------------------------------------------
 * Copia manual para Arduino IDE de glovarm_firmware/glovarm/src/main_glove.cpp
 * (proyecto fuente = PlatformIO). Si se edita acá, replicar el cambio
 * también en el .cpp de PlatformIO — no hay symlink entre ambos.
 *
 * *** ESTA ES LA VERSION QUE HAY QUE USAR PARA CARGAR DE VERDAD ***
 * PlatformIO compila este proyecto sin error, pero ESP-NOW no
 * funciona con la plataforma que trae (confirmado 2026-09-02, ver
 * comentario en platformio.ini). Cargar desde acá, con Arduino IDE
 * o arduino-cli y el core esp32 3.3.11 o mas nuevo.
 *
 * Configuración necesaria en Arduino IDE (Tools):
 *   - Board: "ESP32C3 Dev Module" (paquete esp32 by Espressif Systems, 3.3.11+)
 *   - USB CDC On Boot: "Enabled"  <- IMPRESCINDIBLE en placas Super Mini
 *     (USB nativo sin puente UART): sin esto, Serial.print() no sale
 *     por el puerto USB.
 *   - Librería: "MPU6050" de Electronic Cats (Sketch > Include Library >
 *     Manage Libraries), da I2Cdev.h y MPU6050.h.
 *
 * Callbacks de ESP-NOW: el código soporta tanto el core viejo (2.x)
 * como el nuevo (3.x) via #if ESP_ARDUINO_VERSION_MAJOR.
 *
 * Base: código recuperado del backup (Gemini). Struct de datos
 * centralizado en data_packet.h para no desincronizar con el
 * receptor (ESP-NOW copia el bloque de memoria tal cual).
 *
 * Modo de bajo consumo (confirmado con Agustín, audio 2026-08-19):
 * el guante arranca en IDLE — el hardware (I2C, MPU6050, calibración
 * de giróscopo) se inicializa igual que antes, pero el loop() NO lee
 * sensores ni transmite por ESP-NOW hasta recibir CTRL_CMD_WAKE del
 * receptor. Al recibir CTRL_CMD_SLEEP vuelve a IDLE. Esto evita el
 * consumo dominante (I2C a 100 Hz + radio TX a 100 Hz) mientras no
 * hay operador vinculado, sin necesidad de dormir el radio (que
 * complicaría bastante más la recepción de ESP-NOW — ver nota al
 * final del archivo).
 *
 * NOTA: esta versión no incluye botón de calibración ni LEDs de
 * estado (el Esquema Ordenador los menciona, pero no estaban en el
 * código recuperado). Si los tenían implementados, avisame y los
 * reincorporo.
 */

#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "data_packet.h"

MPU6050 sensor;

// --- DIRECCIÓN MAC DEL RECEPTOR (ESP_brazo) ---
uint8_t broadcastAddress[] = {0x1C, 0xDB, 0xD4, 0xC6, 0x76, 0x38};

// Canal WiFi fijo para ESP-NOW: WiFi.channel() antes de esta llamada
// solo devuelve un valor de configuración, no garantiza que el radio
// ESP-NOW esté realmente sincronizado ahí en las dos puntas — hay que
// fijarlo explícito a nivel driver en ambos nodos (mismo valor acá y
// en main_receiver.cpp) para que los paquetes lleguen de forma confiable.
#define ESPNOW_WIFI_CHANNEL 1

// --- Configuración Sensores Hall ---
const int pinHallIndice  = 0;   // ADC1_CH0
const int pinHallCorazon = 1;   // ADC1_CH1
const int UMBRAL_HALL    = 3000;

int16_t ax, ay, az, gx, gy, gz;
unsigned long tiempo_prev;
float dt;
float angulo_x = 0, angulo_y = 0;
float error_gx = 0, error_gy = 0;

unsigned long tiempoUltimaLectura = 0;
const int INTERVALO_LECTURA = 10; // 100 Hz (transmisión fluida) — solo corre en estado ACTIVO

// --- ESTRUCTURA DE DATOS PARA ESP-NOW (definida en data_packet.h) ---
GloveDataPacket_t datosGuante;
esp_now_peer_info_t peerInfo;

// --- Estado de bajo consumo ---
enum EstadoGuante { GUANTE_IDLE, GUANTE_ACTIVO };
volatile EstadoGuante estadoGuante = GUANTE_IDLE;

// La firma de los callbacks de ESP-NOW cambió entre versiones del
// core arduino-esp32 (PlatformIO usa una vieja tipo IDF4, Arduino IDE
// va quedando en una nueva tipo IDF5 cada vez que se actualiza el
// core). Se compila una firma u otra según la versión detectada, para
// que el mismo .cpp/.ino sirva en ambos toolchains sin tocar nada.
#if ESP_ARDUINO_VERSION_MAJOR >= 3

// Callback para verificar si el envío fue exitoso (depuración).
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // Se puede usar para chequear pérdidas de paquetes si fuese necesario.
}

// Callback de recepción: solo se usa para los mensajes de control
// (CtrlMessage_t, 1 byte) que manda el receptor. Se distingue del
// paquete de sensores por tamaño — el guante nunca debería recibir
// un GloveDataPacket_t, así que cualquier `len` distinto de
// sizeof(CtrlMessage_t) se descarta.
void OnCtrlRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len != sizeof(CtrlMessage_t)) {
    return;
  }
  CtrlMessage_t msg;
  memcpy(&msg, incomingData, sizeof(msg));

  if (msg.cmd == CTRL_CMD_WAKE) {
    estadoGuante = GUANTE_ACTIVO;
    tiempoUltimaLectura = millis(); // evita un primer intervalo "atrasado"
  } else if (msg.cmd == CTRL_CMD_SLEEP) {
    estadoGuante = GUANTE_IDLE;
  }
}

#else

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

void OnCtrlRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  if (len != sizeof(CtrlMessage_t)) {
    return;
  }
  CtrlMessage_t msg;
  memcpy(&msg, incomingData, sizeof(msg));

  if (msg.cmd == CTRL_CMD_WAKE) {
    estadoGuante = GUANTE_ACTIVO;
    tiempoUltimaLectura = millis();
  } else if (msg.cmd == CTRL_CMD_SLEEP) {
    estadoGuante = GUANTE_IDLE;
  }
}

#endif

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9); // Pines I2C ESP32-C3 (SDA=8, SCL=9)

  sensor.initialize();
  if (!sensor.testConnection()) {
    Serial.println("Error al iniciar MPU6050.");
  }

  // --- Calibración giroscopio (offset estático, sensor en reposo) ---
  // Se hace una sola vez al arrancar, independientemente del estado
  // IDLE/ACTIVO, porque requiere que la mano esté quieta.
  delay(1000);
  for (int i = 0; i < 200; i++) {
    sensor.getRotation(&gx, &gy, &gz);
    error_gx += gx; error_gy += gy;
    delay(10);
  }
  error_gx /= 200.0; error_gy /= 200.0;
  tiempo_prev = millis();

  // --- CONFIGURACIÓN WI-FI & ESP-NOW ---
  WiFi.mode(WIFI_STA); // Modo Estación requerido para ESP-NOW
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
  // En IDLE no se lee ni transmite nada: es la parte que ahorra
  // batería (sin esto, el I2C al MPU6050 y el TX por ESP-NOW corren
  // a 100 Hz todo el tiempo, vinculado o no).
  if (estadoGuante != GUANTE_ACTIVO) {
    return;
  }

  unsigned long tiempoActual = millis();

  // TAREA DE LECTURA, EMPAQUETADO Y ENVÍO (cada 10 ms)
  if (tiempoActual - tiempoUltimaLectura >= INTERVALO_LECTURA) {
    tiempoUltimaLectura = tiempoActual;

    // 1. Procesar MPU6050
    sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    dt = tiempoActual - tiempo_prev;
    tiempo_prev = tiempoActual;

    float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14159);
    float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14159);

    float girosc_tasa_x = (gx - error_gx) / 131.0; // 131 LSB/(°/s) @ ±250°/s (datasheet MPU-6050)
    float girosc_tasa_y = (gy - error_gy) / 131.0;

    // Filtro complementario: 96% giróscopo (integración, deriva a
    // largo plazo) + 4% acelerómetro (referencia absoluta, ruido
    // vibratorio a corto plazo).
    angulo_x = 0.96 * (angulo_x + (girosc_tasa_x * (dt / 1000.0))) + 0.04 * accel_ang_x;
    angulo_y = 0.96 * (angulo_y + (girosc_tasa_y * (dt / 1000.0))) + 0.04 * accel_ang_y;

    // 2. Procesar Sensores Hall
    int lecturaIndice  = analogRead(pinHallIndice);
    int lecturaCorazon = analogRead(pinHallCorazon);

    // 3. Cargar el Struct
    datosGuante.angulo_x     = angulo_x;
    datosGuante.angulo_y     = angulo_y;
    datosGuante.pinzaIndice  = (lecturaIndice  > UMBRAL_HALL);
    datosGuante.pinzaCorazon = (lecturaCorazon > UMBRAL_HALL);

    // 4. TRANSMISIÓN INALÁMBRICA INMEDIATA
    esp_err_t resultado = esp_now_send(broadcastAddress,
                                        (uint8_t *)&datosGuante, sizeof(datosGuante));
    (void)resultado; // ver OnDataSent() para diagnóstico asíncrono real
  }
}

/*
 * Nota sobre ahorro de energía adicional (no implementado):
 * -----------------------------------------------------------------
 * Lo de arriba ya elimina la actividad de I2C y de radio TX mientras
 * está IDLE, que es el consumo dominante. Ir más allá — apagar el
 * radio WiFi/PHY (light sleep / modem sleep de verdad) y seguir
 * pudiendo "despertar" al recibir un paquete ESP-NOW — es bastante
 * más delicado: hay que configurar esp_wifi_set_ps() y coordinar el
 * wake source con cuidado para no perder el primer paquete de wake,
 * y con el framework Arduino el soporte es menos directo que en
 * ESP-IDF puro. Dado el tiempo que queda hasta la entrega (30/8), lo
 * dejé afuera; si quieren perseguirlo, mejor probarlo aparte y no
 * sobre el firmware que van a entregar.
 */
