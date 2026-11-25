// =============================================================================
//  config.h — TODOS los parámetros del ordenador de vuelo en un solo sitio
// =============================================================================
//  Idea: igual que en una mesa de luces tienes el "patch" separado del show,
//  aquí separamos "qué hay conectado y cómo" (este archivo) de "qué hace el
//  programa" (el resto). Si cambias un pin o un umbral, solo tocas aquí.
// =============================================================================
#pragma once
#include <Arduino.h>

// ----------------------------------------------------------------- Pines Heltec V3
// Los mismos que en tu transmisor de telemetria-868 (y en tu PCB).
constexpr int PIN_VEXT     = 36;  // a nivel BAJO enciende la salida Vext (OLED + pines Ve)
constexpr int PIN_SENS_SDA = 41;  // bus I2C de los SENSORES (Wire1): MPU6050 + BME280
constexpr int PIN_SENS_SCL = 42;  //   (el 17/18 es el bus interno de la OLED y no sale a los pines)
constexpr int PIN_BOTON    = 0;   // botón "PRG" de la placa
constexpr int PIN_LED      = 35;  // LED blanco de la placa

// GPS por UART (Serial1), los mismos pines que en tu transmisor
constexpr int PIN_GPS_RX  = 47;   // aquí llega el TX del GPS
constexpr int PIN_GPS_TX  = 48;   // va al RX del GPS
constexpr int GPS_BAUDIOS = 9600; // velocidad de fábrica de NEO-6M, NEO-M8N, ATGM336H...

constexpr uint32_t I2C_FRECUENCIA_HZ = 400000;  // I2C "fast mode": 400 kHz
constexpr uint8_t  MPU6050_DIRECCION = 0x68;    // 0x69 si el pin AD0 está a 3,3 V

// ------------------------------------------------------------ Rangos del MPU6050
// Es como el "gain staging" en sonido: más rango = más headroom (no satura
// con el motor o girando rápido) pero cada "escalón" del conversor vale más,
// así que hay más ruido de cuantificación. Para VOLAR: rangos máximos.
//   Giróscopo:    250 | 500 | 1000 | 2000 °/s   (2000 -> 1 bit = 0,061 °/s)
//   Acelerómetro:   2 |   4 |    8 |   16 g     (16   -> 1 bit = 0,49 mg; un motor de modelismo da 10-15 g)
// En tu transmisor usabas 250 °/s y 2 g: más finos para la mesa, pero saturan en vuelo.
constexpr int RANGO_GIRO_DPS = 2000;
constexpr int RANGO_ACEL_G   = 16;

// Filtro paso bajo interno del chip (DLPF, registro 0x1A = 3 -> ~44 Hz).
// Debe quedar por debajo de la mitad de la frecuencia de muestreo (Nyquist:
// 200 Hz / 2 = 100 Hz); si no, las vibraciones rápidas se "doblan" y
// aparecen como movimientos lentos falsos (aliasing).
constexpr uint8_t FILTRO_DLPF = 0x03;

// ------------------------------------------------------------------- Tiempos
constexpr uint32_t FRECUENCIA_IMU_HZ  = 200; // lecturas + filtro por segundo
constexpr uint32_t FRECUENCIA_WEB_HZ  = 50;  // envíos de orientación a la web por segundo
constexpr uint32_t FRECUENCIA_BARO_HZ = 20;  // lecturas del barómetro por segundo (mide a ~23 Hz)
constexpr uint32_t ENVIO_BARO_HZ      = 10;  // envíos del barómetro a la web por segundo

// --------------------------------------------------------------- Calibración
// La placa debe estar QUIETA mientras se mide. Primero se esperan unas
// muestras para que pase el "golpe" de pulsar el botón y luego se promedia.
constexpr uint16_t MUESTRAS_ASENTAMIENTO = 100;  // 0,5 s a 200 Hz (se tiran)
constexpr uint16_t MUESTRAS_CALIBRACION  = 600;  // 3 s a 200 Hz (se promedian)

// Si la desviación típica de alguna lectura supera esto, la placa se ha
// movido (o vibra) durante la medida y la calibración se rechaza.
// Quieto, el ruido del MPU6050 ronda 0,05-0,1 °/s; en la mano son varios °/s.
constexpr float UMBRAL_QUIETUD_DPS = 0.5f;

// El datasheet del MPU6050 garantiza un sesgo de fábrica de como mucho
// ±20 °/s. Un valor mayor indica sensor dañado o un clon raro.
constexpr float SESGO_MAXIMO_DPS = 20.0f;

// true  -> calibra SIEMPRE al encender (lo típico en la rampa de lanzamiento,
//          porque el sesgo cambia con la temperatura).
// false -> usa la calibración guardada; solo calibra si no hay ninguna.
constexpr bool CALIBRAR_AL_ARRANCAR = false;

// --------------------------------------------------------- Filtro de Madgwick
// beta = cuánto "tira" el acelerómetro (la gravedad) para corregir la deriva
// del giróscopo. Se puede cambiar en directo desde la web.
constexpr float BETA_INICIAL = 0.10f;

// Si el módulo de la aceleración se aleja de 1 g más que esto, el
// acelerómetro no está midiendo solo la gravedad (motor encendido, sacudida)
// y ese instante se integra solo con el giróscopo.
constexpr float TOLERANCIA_GRAVEDAD_G = 0.15f;

// ------------------------------------------------------------------- WiFi
// La placa crea su propia red: conéctate a ella y abre http://192.168.4.1
const char* const WIFI_NOMBRE     = "OrdenadorVuelo";
const char* const WIFI_CLAVE      = "cohete1234";   // mínimo 8 caracteres
const char* const NOMBRE_MDNS     = "vuelo";        // -> http://vuelo.local
