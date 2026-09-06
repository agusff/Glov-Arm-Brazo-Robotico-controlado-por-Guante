/*
 * Glov-Arm — Firmware del RECEPTOR (nodo ESP32-C3 en el brazo)
 * ---------------------------------------------------------------
 * Copia manual para Arduino IDE de glovarm_firmware/glovarm/src/main_receiver.cpp
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
 *   - USB CDC On Boot: "Disabled" (default) — a propósito, NO tocar.
 *     Acá Serial es el puente UART real hacia la STM32 por los pines
 *     físicos TX/RX; si se habilita CDC on boot, Serial pasa a ser el
 *     puerto USB y se rompe la conexión con la STM32.
 *
 * Callbacks de ESP-NOW: soporta tanto el core viejo (2.x) como el
 * nuevo (3.x) via #if ESP_ARDUINO_VERSION_MAJOR.
 *
 * Base: código recuperado del backup (Gemini), con la recepción
 * ESP-NOW intacta (mismo struct que el guante, ver data_packet.h).
 *
 * Puente hacia la STM32: trama binaria de 8 bytes con checksum
 * (ver data_packet.h), transmitida siempre cada FRAME_PERIOD_MS —
 * la STM32 decide cuándo actuar sobre esas tramas con su propia
 * máquina de estados (botón: pulsación corta = modo prueba,
 * pulsación larga >2 s = vincula/desvincula el control remoto).
 * Ver la decisión completa de diseño en el README del proyecto.
 *
 * NUEVO — modo de bajo consumo del guante (audio Agustín, 2026-08-19):
 * el receptor le avisa al guante cuándo pasar a ACTIVO (leer y
 * transmitir sensores) y cuándo volver a IDLE (ahorro de batería),
 * disparado por dos comandos puntuales que manda la STM32 por UART
 * en el momento de vincular/desvincular (no en cada trama).
 */

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "data_packet.h"

// Canal WiFi fijo para ESP-NOW — debe coincidir con ESPNOW_WIFI_CHANNEL
// en main_glove.cpp. Ver esa nota para el por qué (WiFi.channel() solo
// consulta configuración, no fuerza el canal real del radio ESP-NOW).
#define ESPNOW_WIFI_CHANNEL 1

// Si no llega un paquete ESP-NOW nuevo del guante en este intervalo,
// se considera el enlace caído (guante transmite cada 10 ms cuando
// está ACTIVO; 150 ms da margen a pérdidas ocasionales de paquete).
#define LINK_TIMEOUT_MS     150

// Período de reenvío de la trama hacia la STM32.
#define FRAME_PERIOD_MS     20

// --- MAC del ESP32 del guante, para poder mandarle mensajes de control ---
uint8_t gloveAddress[] = {0x1C, 0xDB, 0xD4, 0xC6, 0x76, 0x30};
static esp_now_peer_info_t glovePeerInfo;

// --- EL STRUCT DEBE SER IDÉNTICO AL DEL EMISOR (data_packet.h) ---
static GloveDataPacket_t datosRecibidos = {};
static volatile uint32_t lastRxTimestamp = 0;
static volatile bool     hasReceivedOnce = false;

// --- VARIABLES DE LA MÁQUINA DE ESTADOS (RX desde STM32) ---
enum RxState { WAIT_START, WAIT_CMD, WAIT_END };
RxState rx_state = WAIT_START;
uint8_t pending_cmd = 0;


// La firma de los callbacks de ESP-NOW cambió entre versiones del core
// arduino-esp32 (ver misma nota en main_glove.cpp) — se compila una
// firma u otra según la versión detectada.
#if ESP_ARDUINO_VERSION_MAJOR >= 3

// Función Callback que se ejecuta automáticamente al recibir datos por RF.
// Se distingue del futuro tráfico de control por tamaño: el receptor
// solo espera GloveDataPacket_t desde el guante.
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len != sizeof(GloveDataPacket_t)) {
    return; // paquete de tamaño inesperado, se descarta
  }
  memcpy(&datosRecibidos, incomingData, sizeof(datosRecibidos));
  lastRxTimestamp = millis();
  hasReceivedOnce = true;
}

#else

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(GloveDataPacket_t)) {
    return;
  }
  memcpy(&datosRecibidos, incomingData, sizeof(datosRecibidos));
  lastRxTimestamp = millis();
  hasReceivedOnce = true;
}

#endif

static uint8_t currentLinkStatus() {
  if (!hasReceivedOnce) return 0x00;
  return (millis() - lastRxTimestamp <= LINK_TIMEOUT_MS) ? 0x01 : 0x00;
}

// Arma y transmite la trama de 8 bytes hacia la Black Pill.
static void sendFrameToStm32() {
  UartFrame_t frame;
  buildUartFrame(frame, datosRecibidos, currentLinkStatus());
  Serial0.write((uint8_t *)&frame, sizeof(frame)); // Usamos Serial0 para los pines físicos
}
// Manda al guante el comando de despertar/dormir (ver data_packet.h).
static void sendCtrlToGlove(uint8_t cmd) {
  CtrlMessage_t msg;
  msg.cmd = cmd;
  esp_now_send(gloveAddress, (uint8_t *)&msg, sizeof(msg));
}

void setup() {
// 1. Puerto de Depuración (USB hacia la PC)
  Serial.begin(115200);

  // 2. Puerto de Hardware (UART real hacia la STM32 por pines físicos)
  Serial0.begin(115200);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW en el Receptor");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // Peer del guante, necesario para poder mandarle CTRL_CMD_WAKE/SLEEP
  // (recibir datos del guante NO requiere esto, pero enviarle sí).
  memcpy(glovePeerInfo.peer_addr, gloveAddress, 6);
  glovePeerInfo.channel = 0;
  glovePeerInfo.encrypt = false;
  if (esp_now_add_peer(&glovePeerInfo) != ESP_OK) {
    Serial.println("Error al añadir el guante (Peer)");
  }
  Serial.println(">>> ESP32-C3 Gateway Iniciado. Escuchando bus STM32... <<<");
}

void loop() {
  static uint32_t lastFrameSent = 0;

// --- 1. MÁQUINA DE ESTADOS: Recepción desde la STM32 ---
  while (Serial0.available() > 0) {
    uint8_t incoming_byte = Serial0.read();
    
    // Imprimimos en la PC todo lo que entra por el pin físico
    Serial.printf("Byte RX: 0x%02X | Estado FSM antes de procesar: %d\n", incoming_byte, rx_state);
    switch (rx_state) {
      case WAIT_START:
        if (incoming_byte == 0xBB) rx_state = WAIT_CMD;
        break;

      case WAIT_CMD:
        pending_cmd = incoming_byte;
        rx_state = WAIT_END;
        break;

      case WAIT_END:
        if (incoming_byte == 0x55) {
          if (pending_cmd == UART_CMD_VINCULAR) {
            Serial.println("==== COMANDO ACEPTADO: VINCULAR (Despertando Guante) ====");
            sendCtrlToGlove(CTRL_CMD_WAKE);
          } else if (pending_cmd == UART_CMD_DESVINCULAR) {
            Serial.println("==== COMANDO ACEPTADO: DESVINCULAR (Durmiendo Guante) ====");
            sendCtrlToGlove(CTRL_CMD_SLEEP);
          } else {
            Serial.printf("==== ERROR: Comando interno 0x%02X desconocido ====\n", pending_cmd);
          }
        } else {
          Serial.println("==== ERROR: Trama corrupta (Faltó el 0x55 final) ====");
        }
        rx_state = WAIT_START;
        break;
    }
  }
  // --- Streaming periódico de telemetría / estado hacia la STM32 ---
  uint32_t now = millis();
  if (now - lastFrameSent >= FRAME_PERIOD_MS) {
    lastFrameSent = now;
    sendFrameToStm32();
  }
}
