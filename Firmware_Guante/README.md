# 🧤 Nodo Sensor - Guantelete Controlador
### 🌌 Proyecto Glov-Arm

Este directorio contiene el firmware y la especificación técnica del **Nodo Sensor**, integrado directamente en el guantelete del operador. Su función principal es adquirir la biometría gestual en tiempo real, filtrar las señales en el origen (**Edge Computing**) y transmitir los comandos cinemáticos de forma inalámbrica hacia el brazo robótico.

---

## 🛠️ Arquitectura de Hardware

El hardware ha sido optimizado meticulosamente para garantizar un factor de forma compacto y un consumo energético mínimo:

| Componente | Tipo / Bus | Función Cinemática |
| :--- | :--- | :--- |
| **Microcontrolador** | ESP32-C3 Supermini | Cerebro del guante y gestión de transmisión de radio. |
| **Sensor Inercial** | MPU6050 (I2C) | Captura ángulos de **Pitch** (Codo) y **Roll** (Hombro). |
| **Sensores de Flexión** | Efecto Hall (Analógicos) | Mide la flexión del **Índice** (Pinza) y **Corazón** (Muñeca). |

---

## ⚙️ Características Técnicas del Firmware (Edge Computing)

El código procesa la información matemáticamente en el *Edge* antes de transmitirla, liberando de carga computacional al microcontrolador principal del brazo robótico:

* **Máquina de Estados y Bajo Consumo**
  * Implementa estados lógicos estrictos para maximizar la autonomía de la batería.
  * Al encenderse, inicia en modo `GUANTE_IDLE`, suspendiendo el bus I2C y la radio.
  * Solo despierta y transmite al recibir el comando inalámbrico `CTRL_CMD_WAKE` desde el Nodo Enlace.
* **Procesamiento Determinista a 100 Hz**
  * En estado activo, el lazo de lectura de sensores y transmisión por radio se ejecuta fijamente cada **10 milisegundos**.
* **Filtro Complementario Integrado**
  * Elimina el ruido vibratorio y la deriva (*drift*) natural del sensor inercial.
  * Fusiona matemáticamente un **96% de la integración del giroscopio** con un **4% de la referencia del acelerómetro**, obteniendo ángulos absolutos estables.
* **Digitalización Estricta de Flexión**
  * Las lecturas analógicas de los sensores Hall se filtran mediante un comparador por umbral (`UMBRAL_HALL = 3000`).
  * Si la señal supera el límite, se traduce en un estado booleano cerrado, evitando vacilaciones dinámicas en las transiciones de la pinza.

---

## 📡 Protocolo de Comunicación

* **Red Inalámbrica de Baja Latencia:** La estructura de datos se envía utilizando el protocolo **ESP-NOW**.
* **Configuración de Radio:** Opera en un canal WiFi estático (**Canal 1**) para asegurar el sincronismo físico inmediato con el receptor.
* **Direccionamiento:** Los paquetes se transmiten de forma directa apuntando a la dirección MAC de difusión del brazo robótico.

---

## 💻 Entorno de Desarrollo y Notas de Carga

### 🧱 Dependencias y Compilación
* **Entornos Soportados:** Compatible con **PlatformIO** y **Arduino IDE**.
* **Requisito del Core:** Requiere el core de ESP32 en su **versión 3.3.11 o superior**.
* **Librerías Necesarias:** 
  * `MPU6050` (Versión desarrollada por Electronic Cats).
  * `I2Cdev`.

> ⚠️ **Configuración Crítica de Hardware (Arduino IDE):**  
> Para habilitar la depuración por puerto serie en placas **ESP32-C3 Supermini**, es imprescindible ingresar al menú de herramientas y seleccionar la opción **`USB CDC On Boot: Enabled`**. Este microcontrolador utiliza USB nativo y no cuenta con un chip puente UART físico.
