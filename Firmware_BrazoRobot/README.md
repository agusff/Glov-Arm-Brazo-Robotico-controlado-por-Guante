# 🤖 Nodo Actuador - Controlador del Brazo Robotico
### 🌌 Proyecto Glov-Arm

Este directorio contiene el código fuente y la documentación técnica correspondiente al **Nodo Actuador** del sistema de teleoperación. Funciona como el cerebro del brazo robótico, encargado de recibir los comandos de movimiento empaquetados y traducirlos en acciones físicas. Su diseño se centra en un procesamiento determinista y concurrente para replicar con fidelidad el movimiento humano capturado por el guantelete.

---

## 🛠️ Arquitectura de Hardware

El hardware seleccionado garantiza un control robusto, preciso y una retroalimentación en tiempo real para el operador:

| Componente | Especificación / Función |
| :--- | :--- |
| **Microcontrolador** | STM32F411 (Black Pill) como núcleo principal del sistema. |
| **Actuadores** | 4 Servomotores SG90 para el control de pinza, muñeca, codo y hombro. |
| **Interfaz de Usuario** | Pantalla LCD 16x2 con módulo de expansión I2C (PCF8574T). |
| **Control Físico** | Botón dedicado para ejecutar la rutina de puesta a cero y autotesteo. |

### 📺 Estados Visuales (Pantalla LCD)
La pantalla provee retroalimentación visual crítica al operador mostrando los siguientes estados del sistema:
* 🟢 **Conectado:** Enlace activo con el Gateway.
* 🔴 **Desconectado:** Pérdida de comunicación o fuera de línea.
* ⚙️ **Prueba:** Ejecución activa de la rutina *Home*.

> 📐 **Nota sobre el Diseño Mecánico:** El diseño 3D del brazo robótico no es de dominio propio. Si desea conseguir los archivos o ver su ensamblaje, puede obtener más información en este [Video de YouTube](https://www.youtube.com/watch?v=cWuJPlkmxCE).

---

## 💻 Entorno de Desarrollo y Herramientas

* **Lenguaje de Programación:** C nativo enfocado a sistemas embebidos de alto rendimiento.
* **IDE Principal:** [STM32CubeIDE](https://st.com) para la programación y depuración profesional del firmware.
* **Configurador gráfico:** [STM32CubeMX](https://st.com) para la inicialización de periféricos, árbol de relojes (clocks) y mapeo de pines.

---

## ⚙️ Características Técnicas del Firmware

La arquitectura de software está completamente orientada a la eficiencia y a evitar bloqueos de la CPU utilizando técnicas avanzadas de hardware:

* **Control PWM por Hardware**
  * Accionamiento de los SG90 mediante señales PWM generadas directamente por los Timers internos.
  * Garantiza una frecuencia de 50 Hz altamente precisa y libre de fluctuaciones.
  * Elimina por completo el temblor (*jitter*) en los acoples mecánicos.

* **Recepción Asíncrona vía DMA**
  * Gestión de paquetes provenientes del Gateway (vía UART) usando Acceso Directo a Memoria.
  * Permite al microcontrolador leer y validar el *Checksum* de las tramas entrantes en segundo plano.
  * Evita la interrupción del bucle principal y mantiene estables los pulsos de control de los motores.

* **Interfaz Orientada a Eventos**
  * Actualización de la pantalla LCD a través del bus I2C reactiva a cambios de estado.
  * Excluye funciones bloqueantes del bucle principal (*main loop*).
  * Asegura que el flujo de control del brazo nunca pierda prioridad ni sufra retardos.
