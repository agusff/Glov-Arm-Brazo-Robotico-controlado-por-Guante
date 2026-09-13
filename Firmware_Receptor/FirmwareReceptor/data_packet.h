#ifndef DATA_PACKET_H
#define DATA_PACKET_H

#include <cstdint>
#include <cmath>

// ---------------------------------------------------------------
// 1) Paquete ESP-NOW (guante -> receptor)
// ---------------------------------------------------------------
// Ahora transmite exclusivamente ángulos listos para usar (0 a 180)
typedef struct struct_mensaje {
    uint8_t angulo_x;      // Codo (0-180°)
    uint8_t angulo_y;      // Hombro (0-180°)
    uint8_t pinzaIndice;   // Pinza (0-180°)
    uint8_t pinzaCorazon;  // Muñeca (0-180°)
} struct_mensaje;

typedef struct_mensaje GloveDataPacket_t;

// ---------------------------------------------------------------
// 2) Trama UART (receptor -> Black Pill)
// ---------------------------------------------------------------
#define FRAME_START_BYTE   0xAA
#define FRAME_END_BYTE     0x55
#define FRAME_SIZE         8

typedef struct __attribute__((packed)) {
    uint8_t start;          
    uint8_t x_angle;
    uint8_t y_angle;
    uint8_t hall_1;          
    uint8_t hall_2;          
    uint8_t estado_espnow;   
    uint8_t checksum;
    uint8_t end;             
} UartFrame_t;

inline uint8_t computeFrameChecksum(uint8_t x_angle, uint8_t y_angle,
                                     uint8_t hall_1, uint8_t hall_2,
                                     uint8_t estado_espnow) {
    return (uint8_t)(x_angle ^ y_angle ^ hall_1 ^ hall_2 ^ estado_espnow ^ 0xFF);
}

// El Gateway ya no calcula nada, solo traspasa los valores limpios
inline void buildUartFrame(UartFrame_t &frame, const GloveDataPacket_t &data,
                            uint8_t estado_espnow) {
    frame.start        = FRAME_START_BYTE;
    frame.x_angle       = data.angulo_x;
    frame.y_angle       = data.angulo_y;
    frame.hall_1        = data.pinzaIndice;
    frame.hall_2        = data.pinzaCorazon;
    frame.estado_espnow = estado_espnow;
    frame.checksum      = computeFrameChecksum(frame.x_angle, frame.y_angle,
                                                frame.hall_1, frame.hall_2,
                                                estado_espnow);
    frame.end           = FRAME_END_BYTE;
}

// ---------------------------------------------------------------
// 3) Canal de control de bajo consumo
// ---------------------------------------------------------------
typedef struct __attribute__((packed)) {
    uint8_t cmd;
} CtrlMessage_t;

#define CTRL_CMD_SLEEP   0x00   
#define CTRL_CMD_WAKE    0x01   

#define UART_CMD_VINCULAR     0x01   
#define UART_CMD_DESVINCULAR  0x00   

#endif // DATA_PACKET_H