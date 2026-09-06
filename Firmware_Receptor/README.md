Nodo Enlace - Gateway Inalámbrico (Proyecto Glov-Arm)

Descripción General

* Este directorio contiene el código fuente correspondiente al Nodo Enlace (Receptor), ubicado físicamente en la estructura del brazo robótico.


* El dispositivo funciona como una pasarela de comunicación (Gateway) asíncrona y bidireccional entre el guante inalámbrico y el microcontrolador principal del brazo (STM32).


Arquitectura de Hardware

* Microcontrolador: El sistema está basado en una placa ESP32-C3.


* Conectividad: Emplea el radio Wi-Fi interno para la recepción de paquetes ESP-NOW y los pines físicos de hardware (mediante la instancia `Serial0`) para la conexión UART hacia la placa Black Pill.


Características Técnicas del Firmware

La arquitectura de este nodo está diseñada para desvincular los tiempos impredecibles de la red inalámbrica de las estrictas exigencias de tiempo real del control motriz:

* Recepción Inalámbrica Asíncrona: Utiliza interrupciones de hardware (Callbacks) de ESP-NOW para capturar los datos entrantes instantáneamente y registrarlos con una estampa de tiempo (`lastRxTimestamp`) en la memoria sin bloquear el procesador.


* Perro Guardián del Enlace (Watchdog): Incorpora una función de seguridad que evalúa continuamente el tiempo transcurrido desde el último paquete válido recibido. Si se superan los 150 milisegundos (`LINK_TIMEOUT_MS`) sin actividad, el sistema asume una pérdida de enlace y actualiza el byte de estado a desconectado (`0x00`) para detener preventivamente el movimiento del brazo.


* Despacho UART Síncrono a 50 Hz: Independientemente de cómo lleguen los paquetes por radio, el firmware transmite la trama estructurada hacia el STM32 de forma determinista exactamente cada 20 milisegundos. Esto sincroniza la entrega de comandos lógicos con la ventana temporal física de los pulsos PWM requeridos por los servomotores SG90.


* Máquina de Estados de Control (Parser Inverso): El nodo monitorea el bus de recepción UART buscando una trama de 3 bytes (iniciada con `0xBB`) proveniente del STM32. Al recibir y validar comandos de vinculación o desvinculación, el ESP32 transmite mensajes de control (`CTRL_CMD_WAKE` o `CTRL_CMD_SLEEP`) al guante para gestionar su estado de bajo consumo y ahorrar batería.



Entorno de Desarrollo y Notas de Carga

* El código es compatible con el entorno PlatformIO y puede ser compilado directamente en Arduino IDE (requiriendo el core esp32 versión 3.3.11 o superior).


* Canal Wi-Fi Fijo: El canal de radio se configura explícitamente en el Canal 1 a nivel de driver (`esp_wifi_set_channel`) para forzar la sincronización física confiable con el transmisor del guantelete.


* Configuración Crítica (Arduino IDE): Al cargar el firmware en placas ESP32-C3, es imperativo mantener la opción `USB CDC On Boot: Disabled` (por defecto). Esto asegura que la interfaz `Serial` actúe únicamente como puerto de depuración por USB hacia la computadora, permitiendo que la instancia `Serial0` controle los pines físicos TX/RX sin romper la comunicación hacia el STM32.
