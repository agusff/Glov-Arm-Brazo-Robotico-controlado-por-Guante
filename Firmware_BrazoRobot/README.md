---------------Nodo Actuador - Controlador del Brazo Robótico (Proyecto Glov-Arm)------------------------
Descripción General

- Este directorio contiene el código fuente y la documentación técnica correspondiente al Nodo Actuador del sistema de teleoperación.

- Este subsistema funciona como el cerebro del brazo robótico, encargado de recibir los comandos de movimiento empaquetados y traducirlos en acciones físicas.

- El diseño se centra en un procesamiento determinista y concurrente para replicar con fidelidad el movimiento humano capturado por el guantelete.

Arquitectura de Hardware

- El hardware ha sido seleccionado para garantizar un control robusto y preciso:

- Microcontrolador Principal: El núcleo del sistema es una placa de desarrollo STM32F411 (Black Pill).

- Actuadores: El accionamiento mecánico está compuesto por 4 servomotores SG90 que controlan la pinza, muñeca, codo y hombro del brazo.

- Interfaz de Usuario (UI): Se integra una pantalla LCD 16x2 con un módulo de expansión I2C (basado en el chip PCF8574T).

- Estados Visuales: Esta pantalla provee retroalimentación visual al operador mostrando estados críticos: "Conectado", "Desconectado" o "Prueba" (durante la ejecución de la rutina Home).

- Control de Inicialización: Incluye un botón físico dedicado para ejecutar la rutina de puesta a cero y autotesteo de la mecánica.

- El diseño 3D del brazo robot no es de dominio propio, si desea conseguirlo puede obtener informacion en: https://www.youtube.com/watch?v=cWuJPlkmxCE

Entorno de Desarrollo y Herramientas

- Todo el firmware ha sido programado en lenguaje C utilizando el entorno de desarrollo profesional STM32CubeIDE.

- La configuración inicial de los periféricos, el reloj del sistema y el mapeo de pines se realizó mediante la herramienta STM32CubeMX.

Características Técnicas del Firmware

- La arquitectura de software está orientada a la eficiencia y a evitar bloqueos de la CPU:

- Control PWM por Hardware: El accionamiento de los servomotores SG90 se realiza mediante señales PWM generadas directamente por los temporizadores (Timers) de hardware. Esto garantiza una señal de 50 Hz altamente precisa y elimina el temblor ("jitter") en el movimiento.

- Recepción Asíncrona vía DMA: La recepción de los paquetes de datos provenientes del Gateway (vía UART) se gestiona utilizando Acceso Directo a Memoria (DMA). Esto asegura que el microcontrolador pueda leer y validar el Checksum de las tramas entrantes sin interrumpir el procesamiento principal ni los pulsos de control de los motores.

- Interfaz Orientada a Eventos: La actualización de los mensajes en la pantalla LCD a través del bus I2C está diseñada para reaccionar únicamente ante los cambios de estado, evitando funciones bloqueantes dentro del bucle principal que pudieran ralentizar el control del brazo.