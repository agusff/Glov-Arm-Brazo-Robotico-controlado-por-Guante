
# 🦾 Glov-Arm: Brazo Robótico Telemanipulador por Guante

## 📋 Resumen del Proyecto

Glov-Arm es un sistema embebido de telemanipulación que permite controlar un brazo robótico de garra de manera inalámbrica e intuitiva mediante un guante instrumentado. La interfaz gestual elimina la curva de aprendizaje de los controladores convencionales, permitiendo que el robot replique con fidelidad los movimientos naturales de la mano del operador.

## 🧠 Arquitectura de Hardware y Procesamiento Distribuido


1. **Nodo Sensor (Guantelete):**
* **Microcontrolador:** ESP32-C3 Supermini.


* **Adquisición de Datos:** Utiliza un giroscopio MPU6050 (empleando el DMP interno para obtener ángulos limpios) y sensores resistivos de flexión.


* **Procesamiento de Borde:** Aplica un filtro de Media Móvil Exponencial (EMA) para eliminar ruido y libera de carga matemática al controlador principal.


* **Interfaz y Comunicación:** Cuenta con un botón físico para la rutina de calibración ("Puesta a Cero"), LEDs de estado y transmite la biometría vía protocolo inalámbrico ESP-NOW.




2. **Nodo Enlace (Gateway Receptor):**
* **Microcontrolador:** ESP32.


* **Función de Puente:** Actúa como gateway inalámbrico asíncrono recibiendo los paquetes ESP-NOW.


* **Comunicación Interna:** Inyecta los datos al controlador principal mediante un bus UART a alta velocidad (50 Hz), utilizando una trama empaquetada de 8 bytes (Byte inicio + Datos + Estado + Checksum + Byte fin).




3. **Nodo Actuador (Controlador del Brazo):**
* **Microcontrolador Principal:** STM32F411 (Black Pill), programado mediante STM32CubeIDE y STM32CubeMX.


* **Recepción No Bloqueante:** Utiliza Acceso Directo a Memoria (DMA) en modo circular con un Ring Buffer para leer el bus UART sin interrumpir al procesador, validando el Checksum antes de accionar.


* **Actuadores:** Controla 4 servomotores SG90 mediante señales PWM generadas por hardware a través de Timers.


* **Feedback Visual (UI):** Pantalla LCD 16x2 accionada por un módulo I2C (PCF8574T) que se actualiza por eventos mostrando los estados del sistema (Conectado, Desconectado, Prueba), y botón de calibración.


* **Autotesteo:** Al encenderse, ejecuta una rutina preprogramada para verificar el funcionamiento electromecánico antes de aceptar comandos remotos.





## 🖐️ Mapeo Cinemático (Control 1:1)


* **Pinza:** Vinculada a la flexión del dedo índice del guante.


* **Muñeca:** Vinculada a la flexión del dedo medio del guante.


* **Codo:** Vinculado al Pitch (cabeceo) de la muñeca del guante.


* **Hombro:** Vinculado al Roll (alabeo) de la muñeca del guante.



## 🛡️ Robustez y Seguridad (Fail-Safe)


* **Watchdog:** El gateway implementa un temporizador de seguridad de 150 ms. Si la señal inalámbrica se pierde, se aborta la transmisión y los motores se congelan de inmediato en su última posición segura.


* **Interpolación de Trayectoria:** El control genera movimientos fluidos mitigando la inercia dinámica, protegiendo los actuadores de saltos bruscos.


* **Depuración por Hardware-in-the-Loop:** El ESP32 Gateway utiliza bifurcación de puertos (Port Split), separando el USB CDC para debugging en PC, del UART real (pines TX/RX) que va hacia la STM32, evitando la inyección de basura en el bus.



## 🛠️ Herramientas y Tecnologías

* C / C++


* STM32CubeIDE / STM32CubeMX (HAL)


* PlatformIO / VS Code


* Arduino IDE


* Protocolos: UART (DMA), I2C, ESP-NOW

