# 📡 Nodo Enlace - Gateway Inalámbrico
### 🌌 Proyecto Glov-Arm

Este directorio contiene el código fuente correspondiente al **Nodo Enlace (Receptor)**, ubicado físicamente en la estructura del brazo robótico. El dispositivo funciona como una pasarela de comunicación (**Gateway**) asíncrona y bidireccional entre el guante inalámbrico (Nodo Sensor) y el microcontrolador principal del brazo (STM32).

---

## 🛠️ Arquitectura de Hardware

La arquitectura física está diseñada para desvincular los tiempos de la red inalámbrica de las estrictas exigencias de tiempo real del control motriz:

| Componente | Interfaz / Bus | Función Principal |
| :--- | :--- | :--- |
| **Microcontrolador** | ESP32-C3 | Núcleo del Gateway y gestor de doble protocolo. |
| **Conectividad Inalámbrica** | Wi-Fi Interno (Canal 1) | Recepción de paquetes de telemetría vía **ESP-NOW**. |
| **Conectividad Cableada** | UART Hardware (`Serial0`) | Enlace directo hacia la placa Black Pill (STM32). |

---

## ⚙️ Características Técnicas del Firmware

* **Recepción Inalámbrica Asíncrona**
  * Utiliza interrupciones de hardware (*Callbacks*) de ESP-NOW para capturar los datos entrantes instantáneamente.
  * Registra cada paquete con una estampa de tiempo (`lastRxTimestamp`) en memoria sin bloquear el procesador.

* **Perro Guardián del Enlace (Watchdog)**
  * Evalúa continuamente el tiempo transcurrido desde el último paquete válido.
  * Si se superan los **150 milisegundos** (`LINK_TIMEOUT_MS`) sin actividad, el sistema asume una pérdida de enlace.
  * Actualiza preventivamente el byte de estado a desconectado (`0x00`) para detener de inmediato el brazo robótico.

* **Despacho UART Síncrono a 50 Hz**
  * Independientemente de la llegada de paquetes por radio, el firmware transmite la trama estructurada hacia el STM32 de forma determinista exactamente cada **20 milisegundos**.
  * Sincroniza la entrega de comandos con la ventana temporal física de las señales PWM (50 Hz) de los servomotores SG90.

* **Máquina de Estados de Control (Parser Inverso)**
  * Monitorea el bus de recepción UART buscando una trama de 3 bytes (iniciada con el identificador `0xBB`) enviada por el STM32.
  * Al validar comandos de vinculación o desvinculación, transmite mensajes de control (`CTRL_CMD_WAKE` o `CTRL_CMD_SLEEP`) al guante para gestionar su estado de bajo consumo y optimizar la batería.

---

## 💻 Entorno de Desarrollo y Notas de Carga

### 🧱 Dependencias y Compilación
* **Entornos Soportados:** Totalmente compatible con **PlatformIO** y **Arduino IDE**.
* **Requisito del Core:** Requiere el core oficial de ESP32 en su **versión 3.3.11 o superior**.
* **Sincronización de Radio:** El canal se fuerza explícitamente en el **Canal 1** a nivel de driver (`esp_wifi_set_channel`) para garantizar el acoplamiento físico inmediato con el guantelete.

> ⚠️ **Configuración Crítica de Hardware (Arduino IDE):**  
> Al cargar el firmware en la placa ESP32-C3, es imperativo mantener la opción **`USB CDC On Boot: Disabled`** (por defecto). Esto asegura que la interfaz virtual `Serial` actúe únicamente como puerto de depuración por USB hacia la PC, permitiendo que la instancia de hardware `Serial0` controle los pines físicos TX/RX asignados hacia el STM32 sin generar conflictos en la línea de datos.
