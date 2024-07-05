/**
 * @version "v3.0"
 */

/* Includes ---------------------------------------------------------------- */
#include <Lumos_inferencing.h>  // Incluye la biblioteca de inferencia Lumos
#include "freertos/FreeRTOS.h"  // Incluye las bibliotecas FreeRTOS necesarias
#include "freertos/task.h"
#include "driver/adc.h"         // Incluye la biblioteca para el control del ADC
#include "esp_adc_cal.h"        // Incluye la biblioteca para la calibración del ADC
#include "driver/i2s.h"         // Incluye la biblioteca para el control del I2S
#include <WiFi.h>
#include <esp_now.h>

/** Audio buffers, pointers and selectors */
typedef struct {
    int16_t *buffer;           // Buffer para almacenar muestras de audio
    uint8_t buf_ready;         // Indicador de buffer listo para inferencia
    uint32_t buf_count;        // Contador de muestras en el buffer
    uint32_t n_samples;        // Número de muestras por ventana de inferencia
} inference_t;

static inference_t inference;  // Instancia de la estructura de inferencia
static const uint32_t sample_buffer_size = 2048;  // Tamaño del buffer de muestras
static signed short sampleBuffer[sample_buffer_size];  // Buffer para almacenar las muestras de audio
static bool debug_nn = false;  // Cambiar a true para imprimir características generadas
static bool record_status = true;  // Estado de grabación de muestras de audio

/**
 * @brief Dirección MAC del receptor (ESP8266).
 * @note Reemplaza con la dirección MAC del ESP8266.
 */
uint8_t direccionMACReceptor[] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};

/**
 * @brief Etiqueta de inferencia que se enviará.
 */
char etiqueta[32];

/**
 * @brief Callback de envío, se llama cuando un mensaje se ha enviado.
 * 
 * @param mac Dirección MAC del receptor.
 * @param status Estado del envío.
 */
void enviarCallback(const uint8_t* mac, esp_now_send_status_t status) {
    Serial.print("Envio a: "); // Imprime mensaje de inicio del envío
    for (int i = 0; i < 6; i++) {
        Serial.print(mac[i], HEX);// Imprime cada byte de la dirección MAC en formato
        if (i < 5) Serial.print(":");// Agrega ":" entre los bytes, excepto en el último
    }
    Serial.print(" estado: ");// Imprime mensaje de estado
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Éxito" : "Fallo");// Imprime si el envío fue exitoso o falló
}

/**
 * @brief Configuración del ADC e I2S
 */
void adc_init() {
    // Configurar el ADC1 en el canal 6 (GPIO34) con 12 bits de precisión y atenuación de 12 dB
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12);

    // Configuración del I2S
i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN), // Modo maestro, recepción y ADC integrado
    .sample_rate = 16000,  // Tasa de muestreo de 16 kHz
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // 16 bits por muestra
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Solo canal izquierdo
    .communication_format = I2S_COMM_FORMAT_I2S_MSB, // Formato de comunicación MSB
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // Asigna la bandera de interrupción de nivel 1
    .dma_buf_count = 4, // Cantidad de buffers DMA
    .dma_buf_len = 1024, // Longitud del buffer DMA
    .use_apll = false // No utilizar APLL
};
    // Configurar el ADC1 en el canal 6 (GPIO34)
    /* @note  ADC1: 8 channels: GPIO32 - GPIO39ADC1 channel GPIO pin  ADC2 channel  GPIO pin
     *        ADC2: 10 channels: GPIO0, GPIO2, GPIO4, GPIO12 - GPIO15, GOIO25 - GPIO27
     *        ADC1_CHANNEL_0  GPIO36  ADC2_CHANNEL_0  GPIO4
     *        ADC1_CHANNEL_1  GPIO37  ADC2_CHANNEL_1  GPIO0
     *        ADC1_CHANNEL_2  GPIO38  ADC2_CHANNEL_2  GPIO2
     *        ADC1_CHANNEL_3  GPIO39  ADC2_CHANNEL_3  GPIO15
     *        ADC1_CHANNEL_4  GPIO32  ADC2_CHANNEL_4  GPIO13
     *        ADC1_CHANNEL_5  GPIO33  ADC2_CHANNEL_5  GPIO12
     *        ADC1_CHANNEL_6  GPIO34  ADC2_CHANNEL_6  GPIO14
     *        ADC1_CHANNEL_7  GPIO35  ADC2_CHANNEL_7  GPIO27
     *        ADC2_CHANNEL_8  GPIO25
     *        ADC2_CHANNEL_9  GPIO26
     *
     *        Dado que el módulo ADC2 también es utilizado por el Wi-Fi, solo uno de ellos podría obtener la prioridad al usarlos simultáneamente. 
     *        Esto significa que la función adc2_get_raw() podría quedar bloqueada hasta que el Wi-Fi deje de usarse, y viceversa.
    */
    // Configurar I2S
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL); // Instala el controlador I2S
  i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_6); // Establece el modo ADC para el canal 6
  i2s_adc_enable(I2S_NUM_0); // Habilita el ADC para el I2S
}

/**
 * @brief Configuración inicial de Arduino
 */
void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Configurar ADC e I2S
    adc_init();

    // Inicialización del WiFi en modo STA
    WiFi.mode(WIFI_STA);

    // Inicialización del protocolo ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error al inicializar ESP-NOW");
        return;
    }

    // Configuración del callback para la finalización de envíos
    esp_now_register_send_cb(enviarCallback);

    // Añadir el receptor (ESP8266) como par de comunicación
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, direccionMACReceptor, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Error al añadir par");
        return;
    }

    // Imprimir resumen de configuraciones de inferencia
        ei_printf("Configuraciones de inferencia:\n"); // Imprime el encabezado para mostrar las configuraciones de inferencia
        ei_printf("\tIntervalo: "); // Imprime el texto para el intervalo
        ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS); // Imprime el valor del intervalo en milisegundos
        ei_printf(" ms.\n"); // Imprime la unidad de medida del intervalo
        ei_printf("\tTamaño del cuadro: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE); // Imprime el tamaño del cuadro de entrada
        ei_printf("\tDuración de la muestra: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16); // Imprime la duración de la muestra en milisegundos
        ei_printf("\tNúmero de clases: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0])); // Imprime el número de clases para la inferencia

        ei_printf("\nIniciando inferencia continua en 2 segundos...\n"); // Imprime un mensaje indicando el inicio de la inferencia continua
        ei_sleep(2000); // Espera 2 segundos antes de continuar con la ejecución
    // Iniciar la captura de muestras de audio
   // Comprueba si se puede iniciar la inferencia del micrófono con el tamaño de muestra crudo definido
    if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
        // Imprime un mensaje de error si no se puede asignar el búfer de audio con el tamaño especificado
        ei_printf("ERROR: No se pudo asignar el búfer de audio (tamaño %d), esto podría deberse a la longitud de la ventana de su modelo\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        return;
    }
    ei_printf("Grabando...\n");
}

/**
 * @brief se presenta una implementación para realizar la clasificación de audio utilizando Edge Impulse en un dispositivo Arduino. El código se encarga de grabar audio a través de un micrófono, ejecutar el clasificador de Edge Impulse y mostrar las predicciones resultantes.
 * @note Conceptos Clave
    Microphone Inference Record: Función para grabar audio a través del micrófono.
    Run Classifier: Función que ejecuta el clasificador de Edge Impulse.
    Impresión de Resultados: Muestra las predicciones del clasificador y el puntaje de anomalía si está disponible.
    Activación de Pin: Activa o desactiva un pin GPIO según el resultado de la clasificación.
    Estructura del Código
    El código comienza grabando audio a través de la función microphone_inference_record(). Luego, se crea una señal de audio y se ejecuta el clasificador de Edge Impulse con esta señal. Posteriormente, se imprimen las predicciones del clasificador y se activa o desactiva un pin GPIO según el resultado de la clasificación. Finalmente, se resetea el watchdog en cada ciclo de loop.
 */
void loop() {
    bool m = microphone_inference_record(); // Realizar grabación de audio

    if (!m) {
        ei_printf("ERROR: Falló la grabación de audio...\n"); // Imprimir mensaje de error si la grabación falla
        return;
    }

    signal_t signal; // Definir una estructura para almacenar la señal
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT; // Establecer la longitud total de la señal
    signal.get_data = &microphone_audio_signal_get_data; // Obtener los datos de la señal del micrófono

    ei_impulse_result_t result = { 0 }; // Inicializar la estructura de resultados en cero

    // Ejecutar el clasificador de Edge Impulse
    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn); // Llamar a la función para ejecutar el clasificador
    if (r != EI_IMPULSE_OK) {
        ei_printf("ERROR: Falló la ejecución del clasificador (%d)\n", r); // Imprimir mensaje de error si la ejecución del clasificador falla
        return; // Salir de la función si hay un error
    }

    // Imprimir las predicciones del clasificador
    ei_printf("Predicciones "); // Imprime el encabezado de las predicciones
    ei_printf("(DSP: %d ms., Clasificación: %d ms., Anomalía: %d ms.)", // Imprime los tiempos de procesamiento
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n"); // Imprime un salto de línea

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: ", result.classification[ix].label); // Imprime la etiqueta de la clasificación
        ei_printf_float(result.classification[ix].value); // Imprime el valor de la clasificación
        ei_printf("\n"); // Imprime un salto de línea
    }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    puntaje de anomalía: "); // Imprime el puntaje de anomalía
    ei_printf_float(result.anomaly); // Imprime el valor de la anomalía
    ei_printf("\n"); // Imprime un salto de línea
#endif
    digitalWrite(33, LOW);   // Desactivar el pin 33 (GPIO33)

    // Activa el pin 33 si la inferencia es mayor que 0.40000
    if (result.classification[0].value > 0.40000) {  // Comprueba si el valor de clasificación es mayor a 0.4
    ei_printf("    %s: ", result.classification[0].label);  // Imprime la etiqueta de la clasificación
    ei_printf_float(result.classification[0].value);  // Imprime el valor de la clasificación
}
        digitalWrite(33, HIGH);  // Activar el pin 33 (GPIO33)

        // Preparar la etiqueta para enviar
       
       snprintf(etiqueta, sizeof(etiqueta), "%s: %.2f", result.classification[0].label, result.classification[0].value);// Formato de la etiqueta con la clasificación y valor

       // Envío de la etiqueta a través de ESP-NOW
       esp_err_t resultado = esp_now_send(direccionMACReceptor, (uint8_t *)etiqueta, strlen(etiqueta));
       if (resultado == ESP_OK) {
          Serial.println("Etiqueta enviada con éxito"); // Impresión en el puerto serie si la etiqueta se envió con éxito
       } else {
          Serial.println("Error al enviar la etiqueta");// Impresión en el puerto serie si hubo un error al enviar la etiqueta
       }

    // Resetear el watchdog en cada ciclo de loop
    vTaskDelay(pdMS_TO_TICKS(500));
}

/**
 * @brief se presenta una función de callback para la inferencia de audio en un entorno de desarrollo Arduino. La función se encarga de procesar las muestras de audio recibidas y preparar el búfer para la inferencia.
  @note Conceptos Clave
  Callback: Función que se llama en respuesta a un evento específico, en este caso, la llegada de muestras de audio.
  Búfer de Inferencia: Almacena las muestras de audio necesarias para realizar la inferencia.
  n_samples: Número de muestras de audio a procesar en cada iteración.
  Estructura del Código
  La función audio_inference_callback recibe un parámetro n_samples, que representa la cantidad de muestras de audio a procesar. Luego, itera sobre cada muestra recibida y la agrega al búfer de inferencia. Cuando se alcanza la cantidad necesaria de muestras para la inferencia, se reinicia el contador del búfer y se marca como listo para la inferencia.
 */
static void audio_inference_callback(uint32_t n_samples) {
    for(int i = 0; i < n_samples; i++) {
        inference.buffer[inference.buf_count++] = sampleBuffer[i]; // Se añade cada muestra al búfer de inferencia

        if(inference.buf_count >= inference.n_samples) { // Si se alcanza el número de muestras necesario para la inferencia
          inference.buf_count = 0; // Se reinicia el contador del búfer
          inference.buf_ready = 1; // Se indica que el búfer está listo para la inferencia
        }
    }
}

/**
 * @brief Tarea para capturar muestras de audio del ADCEn el fragmento de código proporcionado, se presenta una función en Arduino c++ que se encarga de capturar muestras de audio de un canal específico de un convertidor analógico-digital (ADC). La función opera de manera continua mientras el estado de grabación esté activo, leyendo un número definido de muestras y realizando una llamada a una función de inferencia de audio.
 * @note Conceptos Clave
      samples_to_read: Cantidad de muestras a leer, definida como el argumento recibido por la función.
      sampleBuffer: Arreglo donde se almacenan las muestras de audio.
      adc1_get_raw(ADC1_CHANNEL_6): Función que obtiene la muestra del canal ADC1_CHANNEL_6.
      audio_inference_callback(samples_to_read): Función de inferencia de audio que se llama después de leer las muestras.
      vTaskDelay(pdMS_TO_TICKS(5)): Pausa de 5 milisegundos entre lecturas de muestras.
      vTaskDelete(NULL): Eliminación de la tarea actual al finalizar la captura de muestras.
    Estructura del Código
      La función capture_samples recibe como argumento la cantidad de muestras a leer. Dentro de un bucle while, se lee el número especificado de muestras del canal ADC1_CHANNEL_6 y se almacenan en sampleBuffer. Luego, se verifica el estado de grabación y se llama a la función de inferencia de audio si la grabación está activa. Finalmente, se realiza una pausa de 10 milisegundos antes de repetir el proceso. Una vez que el estado de grabación se desactiva, la tarea se elimina.  
 */
static void capture_samples(void* arg) {
    const uint32_t samples_to_read = (uint32_t)arg;
    size_t bytes_read;

    while (record_status) {
        // Leer datos del I2S
        i2s_read(I2S_NUM_0, (void*)sampleBuffer, samples_to_read * sizeof(int16_t), &bytes_read, portMAX_DELAY);

        if (record_status) {
            // Llamar a la función de callback con el número de muestras leídas
            audio_inference_callback(samples_to_read);
        } else {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
    vTaskDelete(NULL);
}

/**
 * @brief Inicializa la estructura de inferencia y configura/inicia el ADC
 *
 * @param[in] n_samples El número de muestras
 *
 * @return Verdadero si la inicialización fue exitosa, falso de lo contrario
 */
static bool microphone_inference_start(uint32_t n_samples) {
    // Reservar memoria para el búfer de muestras de audio
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));

    // Comprobar si la asignación de memoria fue exitosa
    if(inference.buffer == NULL) {
        return false; // Devolver falso si la asignación falló
    }

    inference.buf_count  = 0; // Inicializar el contador de búfer
    inference.n_samples  = n_samples; // Establecer el número de muestras
    inference.buf_ready  = 0; // Inicializar el indicador de búfer listo

    record_status = true; // Establecer el estado de grabación en verdadero
}


    // Crear tarea para capturar muestras de audio
    xTaskCreate(capture_samples, "CapturarMuestras", 1024 * 32, (void*)sample_buffer_size, 10, NULL);

    return true;
}

/**
 * @brief Espera nuevos datos
 *
 * @return Verdadero cuando finaliza
 */
static bool microphone_inference_record(void) {
    bool ret = true; // Variable para almacenar el valor de retorno

    while (inference.buf_ready == 0) { // Bucle mientras el búfer de inferencia no esté listo
        delay(5); // Esperar 5 milisegundos
    }

    inference.buf_ready = 0; // Reiniciar el indicador de búfer listo
    return ret; // Devolver el valor de retorno
}

/**
 * @brief se presenta una función llamada microphone_audio_signal_get_data que se encarga de convertir muestras de señal de audio de tipo int16 a float. Esta conversión es esencial cuando se trabaja con datos de audio en diferentes formatos dentro de un proyecto de Arduino.
  @note Conceptos Clave
  Conversión de datos de audio de int16 a float.
  Uso de punteros para manipular datos en Arduino.
  Retorno de un valor para indicar el éxito de la operación.
  Estructura del Código
  La función microphone_audio_signal_get_data toma tres parámetros: offset (desplazamiento inicial en el buffer de entrada), length (longitud de datos a procesar) y out_ptr (puntero al buffer de salida donde se almacenarán los datos convertidos).
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    // Convierte los datos de tipo int16 a float
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);
    return 0; // Devuelve 0 al finalizar la función
}

/**
 * @brief Detiene la captura de audio y libera los buffers
 */
static void microphone_inference_end(void) {
    record_status = false;  // Se establece el estado de grabación como falso
    ei_free(inference.buffer);  // Se libera la memoria utilizada por el búfer de inferencia
    i2s_adc_disable(I2S_NUM_0);  // Se deshabilita la entrada de audio ADC
    i2s_driver_uninstall(I2S_NUM_0);  // Se desinstala el controlador I2S
}
