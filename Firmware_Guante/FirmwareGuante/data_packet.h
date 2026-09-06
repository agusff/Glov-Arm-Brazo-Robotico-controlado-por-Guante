#ifndef DATA_PACKET_H
#define DATA_PACKET_H

#include <cstdint>
#include <cmath>

/*
 * Glov-Arm — Definición compartida del protocolo de datos
 * ---------------------------------------------------------------
 * Este archivo se incluye TANTO en el firmware del guante (TX)
 * como en el del receptor (RX).
 *
 * Existen dos capas de datos distintas:
 *
 *  1) Paquete ESP-NOW (guante -> receptor): GloveDataPacket_t.
 *     Es EXACTAMENTE el struct_mensaje del código original
 *     recuperado (2 floats + 2 bool) — no se puede cambiar sin
 *     recompilar y volver a cargar AMBOS nodos, porque ESP-NOW
 *     copia el bloque de memoria tal cual.
 *
 *  2) Trama UART (receptor -> Black Pill / STM32F411): 8 bytes,
 *     formato binario acordado con Agustín (audio 2026-08-19):
 *
 *     frame[8] = {0xAA, x_angle, y_angle, hall_1, hall_2,
 *                 estado_espnow, checksum, 0x55};
 *
 *     checksum = x_angle ^ y_angle ^ hall_1 ^ hall_2
 *                ^ estado_espnow ^ 0xFF;
 *
 *     Reemplaza el formato ASCII "$ang_x,ang_y,pinza1,pinza2*" de
 *     la versión anterior del receptor.
 */

// ---------------------------------------------------------------
// 1) Paquete ESP-NOW (guante -> receptor)
// ---------------------------------------------------------------
// Idéntico en emisor y receptor: ESP-NOW transmite el bloque de
// memoria crudo, por lo que un mismatch de layout corrompe los datos.
typedef struct struct_mensaje {
    float angulo_x;      // Salida del filtro complementario (Codo)
    float angulo_y;      // Salida del filtro complementario (Hombro)
    bool  pinzaIndice;   // Hall dedo índice > UMBRAL_HALL (Pinza)
    bool  pinzaCorazon;  // Hall dedo medio  > UMBRAL_HALL (Muñeca del brazo)
} struct_mensaje;

typedef struct_mensaje GloveDataPacket_t;

// ---------------------------------------------------------------
// 2) Trama UART (receptor -> Black Pill)
// ---------------------------------------------------------------
#define FRAME_START_BYTE   0xAA
#define FRAME_END_BYTE     0x55
#define FRAME_SIZE         8

// Rango angular mapeado al byte 0-255 de la trama. El filtro
// complementario del guante puede entregar valores fuera de este
// rango en movimientos bruscos; se satura (constrain) antes de
// convertir. Ajustar si el recorrido real de muñeca es distinto.
#define ANGLE_RANGE_DEG    90.0f

typedef struct __attribute__((packed)) {
    uint8_t start;          // Byte de sincronismo (0xAA)
    uint8_t x_angle;
    uint8_t y_angle;
    uint8_t hall_1;          // 0x01 = pinza índice activada, 0x00 = no
    uint8_t hall_2;          // 0x01 = pinza corazón activada, 0x00 = no
    uint8_t estado_espnow;   // 0x01 = enlace guante-receptor activo
    uint8_t checksum;
    uint8_t end;             // Byte de fin de trama (0x55)
} UartFrame_t;

// Convierte un ángulo (grados) a byte 0-255, saturando a
// ±ANGLE_RANGE_DEG. 0 = -ANGLE_RANGE_DEG, 255 = +ANGLE_RANGE_DEG.
inline uint8_t angleToByte(float angleDeg) {
    float clamped = angleDeg;
    if (clamped > ANGLE_RANGE_DEG)  clamped = ANGLE_RANGE_DEG;
    if (clamped < -ANGLE_RANGE_DEG) clamped = -ANGLE_RANGE_DEG;
    float normalized = (clamped + ANGLE_RANGE_DEG) / (2.0f * ANGLE_RANGE_DEG); // 0..1
    int v = (int)(normalized * 255.0f + 0.5f);
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    return (uint8_t)v;
}

inline uint8_t computeFrameChecksum(uint8_t x_angle, uint8_t y_angle,
                                     uint8_t hall_1, uint8_t hall_2,
                                     uint8_t estado_espnow) {
    return (uint8_t)(x_angle ^ y_angle ^ hall_1 ^ hall_2 ^ estado_espnow ^ 0xFF);
}

// Arma la trama completa de 8 bytes a partir del último paquete
// ESP-NOW recibido y del estado de enlace calculado por el receptor.
inline void buildUartFrame(UartFrame_t &frame, const GloveDataPacket_t &data,
                            uint8_t estado_espnow) {
    frame.start        = FRAME_START_BYTE;
    frame.x_angle       = angleToByte(data.angulo_x);
    frame.y_angle       = angleToByte(data.angulo_y);
    frame.hall_1        = data.pinzaIndice  ? 0x01 : 0x00;
    frame.hall_2        = data.pinzaCorazon ? 0x01 : 0x00;
    frame.estado_espnow = estado_espnow;
    frame.checksum      = computeFrameChecksum(frame.x_angle, frame.y_angle,
                                                frame.hall_1, frame.hall_2,
                                                estado_espnow);
    frame.end           = FRAME_END_BYTE;
}

// ---------------------------------------------------------------
// 3) Canal de control de bajo consumo: receptor -> guante (ESP-NOW)
// ---------------------------------------------------------------
// Mensaje aparte del paquete de sensores (distinto tamaño en bytes,
// por eso el receptor de cada lado puede distinguirlos por `len` en
// el callback de esp_now sin necesidad de un campo de tipo extra).
//
// Idea (confirmada con Agustín, audio 2026-08-19): el guante arranca
// en IDLE (no lee sensores ni transmite, solo escucha ESP-NOW) para
// ahorrar batería mientras no hay operador vinculado. El receptor le
// avisa cuándo pasar a ACTIVO y cuándo volver a IDLE.
typedef struct __attribute__((packed)) {
    uint8_t cmd;
} CtrlMessage_t;

#define CTRL_CMD_SLEEP   0x00   // -> el guante entra/vuelve a IDLE
#define CTRL_CMD_WAKE    0x01   // -> el guante pasa a ACTIVO (lee y transmite sensores)

// Comandos que la STM32 manda por UART al receptor para disparar el
// mensaje de control de arriba. Solo se envían en el evento de
// presionar el botón de vinculación (>2 s), no en cada trama.
// AJUSTAR según el firmware real de la STM32 — no están definidos en
// la transcripción de audio, son un supuesto de implementación.
#define UART_CMD_VINCULAR     0x10   // STM32 -> receptor: "vinculate y despertá al guante"
#define UART_CMD_DESVINCULAR  0x11   // STM32 -> receptor: "dormí al guante y desvinculate"

#endif // DATA_PACKET_H
