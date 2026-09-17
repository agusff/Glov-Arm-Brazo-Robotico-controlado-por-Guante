/*
 * Glov-Arm — Firmware del RECEPTOR (nodo ESP32-C3 en el brazo)
 * ---------------------------------------------------------------
 *
 * Configuración necesaria en Arduino IDE (Tools):
 *   - Board: "ESP32C3 Dev Module" (paquete esp32 by Espressif Systems, 3.3.11+)
 *   - USB CDC On Boot: "Disabled" (default) — a propósito, NO tocar.
 *     Acá Serial es el puente UART real hacia la STM32 por los pines
 *     físicos TX/RX; si se habilita CDC on boot, Serial pasa a ser el
 *     puerto USB y se rompe la conexión con la STM32.
 */

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "data_packet.h"

#define ESPNOW_WIFI_CHANNEL 1

// Si no llega un paquete ESP-NOW nuevo del guante en este intervalo,
// se considera el enlace caído (guante transmite cada 10 ms cuando
// está ACTIVO; 150 ms da margen a pérdidas ocasionales de paquete).
#define LINK_TIMEOUT_MS     150

// Período de reenvío de la trama hacia la STM32.
#define FRAME_PERIOD_MS     20

// MAC del ESP32 del guante
uint8_t gloveAddress[] = {0x1C, 0xDB, 0xD4, 0xC6, 0x76, 0x30};
static esp_now_peer_info_t glovePeerInfo;

static GloveDataPacket_t datosRecibidos = {};
static volatile uint32_t lastRxTimestamp = 0;
static volatile bool     hasReceivedOnce = false;

// VARIABLES DE LA MÁQUINA DE ESTADOS
enum RxState { WAIT_START, WAIT_CMD, WAIT_END };
RxState rx_state = WAIT_START;
uint8_t pending_cmd = 0;


#if ESP_ARDUINO_VERSION_MAJOR >= 3


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
  Serial0.write((uint8_t *)&frame, sizeof(frame)); // Serial0 para los pines físicos
}
// Manda al guante el comando de despertar/dormir
static void sendCtrlToGlove(uint8_t cmd) {
  CtrlMessage_t msg;
  msg.cmd = cmd;
  esp_now_send(gloveAddress, (uint8_t *)&msg, sizeof(msg));
}

void setup() {
  // Puerto de Depuración (USB hacia la PC)
  Serial.begin(115200);

  // Puerto de Hardware (UART real hacia la STM32 por pines físicos)
  Serial0.begin(115200);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW en el Receptor");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // Peer del guante, necesario para poder mandarle CTRL_CMD_WAKE/SLEEP
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

// MÁQUINA DE ESTADOS: Recepción desde la STM32
  while (Serial0.available() > 0) {
    uint8_t incoming_byte = Serial0.read();
    
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
