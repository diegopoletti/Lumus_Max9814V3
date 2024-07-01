# Lumus_Max9814V3
 Lumus utilizando Max9814 y transmisión ESP-NOW de la Etiqueta seleccionada

Explicación del Código
ESP32 (Maestro)

    Inclusión de Librerías:
        WiFi.h se usa para manejar la conectividad WiFi.
        esp_now.h se usa para manejar el protocolo ESP-NOW.

    Dirección MAC del Receptor y Etiqueta:

    cpp

uint8_t direccionMACReceptor[] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};
char etiqueta[32];

    Define la dirección MAC del ESP8266 receptor.
    Define el buffer para la etiqueta que se enviará.

Callback de Envío:

    Función que se llama cuando se completa un envío, imprimiendo el resultado en el Serial.

Inicialización de ESP-NOW en setup:

    Configura el modo WiFi y ESP-NOW, y añade el receptor como par de comunicación.

Envío de Etiqueta en loop:

    Graba audio, ejecuta la inferencia, y si el resultado es mayor a 0.4, prepara y envía la etiqueta al receptor.
