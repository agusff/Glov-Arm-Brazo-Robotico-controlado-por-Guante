Nodo Sensor - Guantelete Controlador 

Descripción General

* Este directorio contiene el firmware y la especificación técnica del Nodo Sensor, integrado en el guantelete del operador.

* Su función principal es adquirir la biometría gestual en tiempo real, filtrar las señales en el origen (Edge Computing) y transmitir los comandos cinemáticos de forma inalámbrica hacia el brazo robótico.


Arquitectura de Hardware

El hardware ha sido optimizado para ser compacto y eficiente energéticamente:

* Microcontrolador: El sistema está gobernado por una placa ESP32-C3 Supermini.


* Sensor Inercial: Se emplea un módulo MPU6050 comunicado mediante el bus I2C. Este sensor captura los ángulos de cabeceo (Pitch) y alabeo (Roll) de la mano para controlar las articulaciones del codo y el hombro respectivamente.


* Sensores de Flexión: Se utilizan sensores analógicos de efecto Hall para medir la flexión de los dedos índice y corazón, los cuales controlan la pinza y la muñeca.



Características Técnicas del Firmware (Edge Computing)

El código está diseñado para procesar la información matemáticamente antes de transmitirla, liberando de carga al microcontrolador principal del brazo:

* Máquina de Estados y Bajo Consumo: El firmware implementa una máquina de estados para maximizar la autonomía de la batería. Al encenderse, el guante arranca en modo `GUANTE_IDLE`, suspendiendo el bus I2C y la transmisión de radio. Solo comienza a transmitir cuando recibe el comando inalámbrico `CTRL_CMD_WAKE` desde el Nodo Enlace.


* Procesamiento a 100 Hz: En estado activo, el lazo de lectura y transmisión se ejecuta de manera determinista cada 10 milisegundos.


* Filtro Complementario: Para eliminar el ruido vibratorio y la deriva del sensor inercial, el firmware aplica matemáticamente un filtro que fusiona un 96% de la integración del giroscopio con un 4% de la referencia del acelerómetro, logrando ángulos absolutos altamente estables.


* Digitalización de Flexión: Las lecturas analógicas de los sensores Hall se procesan mediante un comparador de umbral (`UMBRAL_HALL = 3000`). Si la señal supera este límite, se traduce en un estado booleano cerrado, asegurando transiciones sin vacilaciones dinámicas.



Protocolo de Comunicación

* Red Inalámbrica de Baja Latencia: La transmisión de la estructura de datos se realiza utilizando el protocolo ESP-NOW configurado en un canal WiFi estático (Canal 1) para asegurar el sincronismo físico con el receptor. Los paquetes se envían directamente a la dirección MAC de difusión del brazo robótico.



Entorno de Desarrollo y Notas de Carga

* El firmware puede ser compilado tanto en PlatformIO como en Arduino IDE** (requiere el core de ESP32 versión 3.3.11 o superior).


* Dependencias: Requiere la instalación de la librería MPU6050 (de Electronic Cats) y I2Cdev.


* Configuración Crítica (Arduino IDE): Para habilitar la depuración por puerto serie en placas ESP32-C3 Supermini, es imprescindible seleccionar la opción `USB CDC On Boot: Enabled` en el menú de herramientas, ya que este microcontrolador utiliza USB nativo sin puente UART.
