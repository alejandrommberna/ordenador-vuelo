// =============================================================================
//  Ordenador de vuelo — IMU calibrada + barómetro + GPS, con web 3D en directo
//  Placa: Heltec WiFi LoRa 32 V3 (ESP32-S3)
//  Sensores (tu PCB): MPU6050 + BME280/BMP280 en I2C (GPIO 41/42), GPS en UART (GPIO 47/48)
// =============================================================================
//  Varias "cadencias" a la vez, cada una a su ritmo (como capas de un show
//  con distinto BPM sobre el mismo reloj):
//    · 200 Hz  IMU: lee el MPU6050, resta el sesgo y pasa el filtro de Madgwick
//    ·  20 Hz  barómetro: presión -> altitud y velocidad vertical
//    ·  siempre GPS: va vaciando las frases NMEA que llegan por el puerto serie
//    ·  50 Hz  web: orientación   |  10 Hz: barómetro  |  1 Hz: GPS
//
//  Cómo usarlo:
//    · Conéctate a la WiFi "OrdenadorVuelo" (clave cohete1234)
//    · Abre http://192.168.4.1  (o http://vuelo.local)
//    · Calibrar: botón en la web, tecla 'c' en el monitor serie, o mantener
//      pulsado el botón PRG de la placa 1 segundo y soltar.
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "imu.h"
#include "madgwick.h"
#include "baro.h"
#include "gps.h"
#include "web.h"

static Imu imu;
static Madgwick filtro;
static Barometro baro;
static Gps gps;

static constexpr float    GRADOS_A_RAD   = 0.01745329252f;
static constexpr uint32_t PERIODO_IMU_US = 1000000UL / FRECUENCIA_IMU_HZ;  // 5000 µs
static constexpr uint32_t PERIODO_WEB_MS = 1000UL / FRECUENCIA_WEB_HZ;     // 20 ms

static float frecuenciaReal_hz = 0;   // cuántas muestras por segundo conseguimos de verdad
static uint32_t erroresI2C = 0;       // lecturas fallidas en el último segundo

// ------------------------------------------------------------- utilidades
static void escanearI2C(TwoWire& bus) {
  Serial.printf("Dispositivos en el bus de sensores (SDA %d / SCL %d):\n", PIN_SENS_SDA, PIN_SENS_SCL);
  int n = 0;
  for (uint8_t direccion = 1; direccion < 127; direccion++) {
    bus.beginTransmission(direccion);
    if (bus.endTransmission() == 0) {
      const char* nombre = direccion == 0x3C ? " (pantalla OLED)"
                         : direccion == 0x68 || direccion == 0x69 ? " (MPU6050)"
                         : direccion == 0x76 || direccion == 0x77 ? " (BME280 / BMP280)" : "";
      Serial.printf("  0x%02X%s\n", direccion, nombre);
      n++;
    }
  }
  if (n == 0) Serial.println("  ninguno: revisa el cableado");
}

static const char* textoEstado(EstadoCalibracion e) {
  switch (e) {
    case EstadoCalibracion::Midiendo: return "midiendo";
    case EstadoCalibracion::Correcta: return "ok";
    case EstadoCalibracion::Fallida:  return "fallo";
    default:                          return "sin";
  }
}

// Pone la orientación a partir de la gravedad, promediando unas cuantas
// lecturas para que el ruido del acelerómetro no la tuerza.
static void alinearConGravedad() {
  float suma[3] = {0, 0, 0};
  const int N = 20;
  for (int i = 0; i < N; i++) {
    imu.leer();
    for (int k = 0; k < 3; k++) suma[k] += imu.acel_g[k];
    delay(5);
  }
  filtro.alinear(suma[0] / N, suma[1] / N, suma[2] / N);
}

// ------------------------------------------------- mensajes hacia la web
// Formato JSON escrito "a mano" con snprintf (no hace falta otra librería).
// Hay dos tipos de mensaje, distinguidos por el campo "t":
//   "c" = estado de la calibración (solo cuando cambia)
//   "d" = datos en directo (50 Hz)

static void enviarEstadoCalibracion() {
  const float* b = imu.sesgo();
  const float* s = imu.desviacion();
  char json[320];
  snprintf(json, sizeof(json),
           "{\"t\":\"c\",\"estado\":\"%s\",\"tiene\":%d,"
           "\"b\":[%.4f,%.4f,%.4f],\"s\":[%.4f,%.4f,%.4f],"
           "\"Tc\":%.2f,\"beta\":%.3f,\"msg\":\"%s\"}",
           textoEstado(imu.estado()), imu.tieneCalibracion() ? 1 : 0,
           b[0], b[1], b[2], s[0], s[1], s[2],
           imu.temperaturaCalibracion(), filtro.beta, imu.mensaje());
  web::enviarATodos(json);
}

static void enviarDatos() {
  char json[256];
  snprintf(json, sizeof(json),
           "{\"t\":\"d\",\"q\":[%.5f,%.5f,%.5f,%.5f],"   // orientación (cuaternión)
           "\"g\":[%.3f,%.3f,%.3f],"                     // giro corregido °/s
           "\"a\":[%.4f,%.4f,%.4f],"                     // aceleración en g
           "\"T\":%.2f,\"hz\":%.0f,\"cp\":%.3f,\"ac\":%d}",
           filtro.q0, filtro.q1, filtro.q2, filtro.q3,
           imu.giro_dps[0], imu.giro_dps[1], imu.giro_dps[2],
           imu.acel_g[0], imu.acel_g[1], imu.acel_g[2],
           imu.temperatura_c, frecuenciaReal_hz, imu.progreso(),
           filtro.usandoGravedad ? 1 : 0);
  web::enviarATodos(json);
}

// Barómetro (10 Hz). Si no hay sensor, solo se dice eso.
static void enviarBarometro() {
  char json[256];
  if (!baro.presente()) {
    snprintf(json, sizeof(json), "{\"t\":\"b\",\"chip\":\"ninguno\"}");
  } else {
    snprintf(json, sizeof(json),
             "{\"t\":\"b\",\"chip\":\"%s\",\"p\":%.3f,\"T\":%.2f,\"hr\":%.1f,"
             "\"h\":%.3f,\"vz\":%.3f,\"hmax\":%.3f,\"hisa\":%.1f}",
             baro.modelo(), baro.presion_hpa, baro.temperatura_c,
             isnan(baro.humedad) ? -1.0f : baro.humedad,
             baro.altitud_m, baro.velocidadVert_ms, baro.altitudMax_m, baro.altitudIsa_m);
  }
  web::enviarATodos(json);
}

// GPS (1 Hz o cuando llega una posición nueva)
static void enviarGps() {
  TinyGPSPlus& g = gps.nmea;
  char hora[12] = "";
  if (g.time.isValid()) snprintf(hora, sizeof(hora), "%02d:%02d:%02d", g.time.hour(), g.time.minute(), g.time.second());
  uint8_t fix = gps.tipoFix();
  char json[320];
  snprintf(json, sizeof(json),
           "{\"t\":\"g\",\"mudo\":%d,\"fix\":%d,\"sat\":%d,\"hdop\":%.2f,"
           "\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f,\"vel\":%.2f,\"rumbo\":%.1f,"
           "\"hora\":\"%s\",\"ok\":%u,\"mal\":%u}",
           gps.mudo() ? 1 : 0, fix,
           g.satellites.isValid() ? (int)g.satellites.value() : 0,
           g.hdop.isValid() ? g.hdop.hdop() : -1.0,
           fix ? g.location.lat() : 0.0, fix ? g.location.lng() : 0.0,
           fix == 3 ? g.altitude.meters() : 0.0,
           g.speed.isValid() ? g.speed.kmph() : 0.0,
           g.course.isValid() ? g.course.deg() : 0.0,
           hora, (unsigned)g.passedChecksum(), (unsigned)g.failedChecksum());
  web::enviarATodos(json);
}

static void empezarCalibracion() {
  imu.empezarCalibracion();
  enviarEstadoCalibracion();   // para que la web muestre "midiendo" ya
}

// --------------------------------------------------- órdenes y entradas
static void atenderOrdenesWeb() {
  web::Mensaje m;
  while (web::siguienteOrden(m)) {
    switch (m.orden) {
      case web::Orden::Calibrar:  empezarCalibracion(); break;
      case web::Orden::Borrar:    imu.borrarCalibracion(); break;   // avisa vía hayNovedad()
      case web::Orden::Realinear: alinearConGravedad(); Serial.println("Realineado con la gravedad"); break;
      case web::Orden::CeroAltitud: baro.ponerACero(); break;
      case web::Orden::CambiarBeta:
        filtro.beta = constrain(m.valor, 0.0f, 1.0f);
        Serial.printf("beta = %.3f\n", filtro.beta);
        enviarEstadoCalibracion();
        break;
    }
  }
  if (web::hayClienteNuevo()) enviarEstadoCalibracion();
}

static void atenderBoton() {
  // Mantener pulsado PRG >= 1 s y SOLTAR -> calibrar.
  // Se dispara al soltar para que el dedo ya no esté tocando la placa.
  static bool estabaPulsado = false;
  static uint32_t pulsadoDesde = 0;
  bool pulsado = digitalRead(PIN_BOTON) == LOW;   // con pull-up: pulsado = 0 V
  if (pulsado && !estabaPulsado) pulsadoDesde = millis();
  if (!pulsado && estabaPulsado && millis() - pulsadoDesde >= 1000) empezarCalibracion();
  estabaPulsado = pulsado;
}

static void atenderSerie() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'c') empezarCalibracion();
    if (c == 'b') imu.borrarCalibracion();
    if (c == 'r') { alinearConGravedad(); Serial.println("Realineado con la gravedad"); }
    if (c == 'z') baro.ponerACero();
  }
}

static void informeSerie() {
  // Ángulos de Euler solo para leerlos en el monitor serie
  float q0 = filtro.q0, q1 = filtro.q1, q2 = filtro.q2, q3 = filtro.q3;
  float roll  = atan2f(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) / GRADOS_A_RAD;
  float pitch = asinf(constrain(2 * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) / GRADOS_A_RAD;
  float yaw   = atan2f(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) / GRADOS_A_RAD;
  Serial.printf("%3.0f Hz | roll %+7.2f  pitch %+7.2f  yaw %+7.2f | giro %+6.2f %+6.2f %+6.2f °/s | cal: %s",
                frecuenciaReal_hz, roll, pitch, yaw,
                imu.giro_dps[0], imu.giro_dps[1], imu.giro_dps[2], textoEstado(imu.estado()));
  if (baro.presente()) Serial.printf(" | alt %+6.2f m  vz %+5.2f m/s", baro.altitud_m, baro.velocidadVert_ms);
  if (gps.mudo()) Serial.print(" | GPS mudo");
  else Serial.printf(" | GPS fix %d, %d sat", gps.tipoFix(),
                     gps.nmea.satellites.isValid() ? (int)gps.nmea.satellites.value() : 0);
  if (erroresI2C) Serial.printf(" | %u lecturas I2C fallidas", (unsigned)erroresI2C);
  Serial.println();
}

// ===================================================================== setup
void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n=== Ordenador de vuelo · Fase 1 (IMU + web 3D) ===");

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BOTON, INPUT_PULLUP);

  // Vext a nivel bajo: da corriente a los pines Ve (por si tu PCB se alimenta de ahí)
  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);
  delay(100);

  // La ESP32-S3 tiene DOS controladores I2C: Wire y Wire1. Como en tu
  // transmisor, Wire queda para la OLED (17/18) y Wire1 para los sensores.
  Wire1.begin(PIN_SENS_SDA, PIN_SENS_SCL, I2C_FRECUENCIA_HZ);
  escanearI2C(Wire1);

  Serial.println("Iniciando MPU6050...");
  while (!imu.iniciar(Wire1, MPU6050_DIRECCION)) {
    Serial.println("  MPU6050 no disponible. Reintento en 2 s");
    digitalWrite(PIN_LED, HIGH); delay(100); digitalWrite(PIN_LED, LOW);
    delay(1900);
  }
  Serial.printf("  MPU6050 OK (±%d °/s, ±%d g, filtro 44 Hz, 200 Hz)\n", RANGO_GIRO_DPS, RANGO_ACEL_G);

  filtro.beta = BETA_INICIAL;
  alinearConGravedad();   // empezamos ya con la inclinación correcta

  Serial.println("Iniciando barómetro...");
  baro.iniciar(Wire1);    // si no está, el resto sigue funcionando
  Serial.println("Iniciando GPS...");
  gps.iniciar();

  Serial.println("Iniciando WiFi y servidor web...");
  web::iniciar();

  if (CALIBRAR_AL_ARRANCAR || !imu.tieneCalibracion()) empezarCalibracion();

  Serial.println("Teclas: c = calibrar, b = borrar calibración, r = realinear, z = altitud a cero\n");
}

// ====================================================================== loop
void loop() {
  static uint32_t proximaMuestra_us  = micros();
  static uint32_t muestraAnterior_us = micros();
  static uint32_t muestrasEnEsteSegundo = 0;
  static uint32_t inicioSegundo_ms = millis();
  static uint32_t ultimoEnvioWeb_ms = 0;
  static uint32_t ultimaLecturaBaro_ms = 0, ultimoEnvioBaro_ms = 0, ultimoEnvioGps_ms = 0;

  // ------------------------------------------- 200 Hz: IMU + filtro
  // En vez de delay(), miramos el reloj: así el resto del loop (web, botón,
  // serie) sigue funcionando entre muestra y muestra.
  uint32_t ahora_us = micros();
  if ((int32_t)(ahora_us - proximaMuestra_us) >= 0) {
    proximaMuestra_us += PERIODO_IMU_US;
    // Si nos hemos quedado muy atrás (algo bloqueó el loop), resincronizamos
    if ((int32_t)(ahora_us - proximaMuestra_us) > (int32_t)PERIODO_IMU_US)
      proximaMuestra_us = ahora_us + PERIODO_IMU_US;

    if (!imu.leer()) {
      // Muestra perdida (I2C): no movemos el filtro; el siguiente dt cubrirá el hueco
      erroresI2C++;
    } else {
      // dt REAL desde la muestra anterior (no el teórico): el filtro integra
      // velocidad x tiempo, así que el tiempo tiene que ser el de verdad.
      float dt = (ahora_us - muestraAnterior_us) * 1e-6f;
      muestraAnterior_us = ahora_us;
      if (dt > 0.05f) dt = 0.05f;   // protección ante un parón largo

      filtro.actualizar(imu.giro_dps[0] * GRADOS_A_RAD,   // el filtro trabaja en rad/s
                        imu.giro_dps[1] * GRADOS_A_RAD,
                        imu.giro_dps[2] * GRADOS_A_RAD,
                        imu.acel_g[0], imu.acel_g[1], imu.acel_g[2],
                        dt, TOLERANCIA_GRAVEDAD_G);
      muestrasEnEsteSegundo++;

      // ¿Acaba de terminar una calibración?
      if (imu.hayNovedad()) {
        if (imu.estado() == EstadoCalibracion::Correcta) filtro.alinear(imu.acel_g[0], imu.acel_g[1], imu.acel_g[2]);
        enviarEstadoCalibracion();
      }
    }
  }

  // ------------------------------------------- GPS: vaciar el buzón serie
  gps.atender();

  // --------------------------------------------------- 20 Hz: barómetro
  uint32_t ahora_ms = millis();
  if (ahora_ms - ultimaLecturaBaro_ms >= 1000 / FRECUENCIA_BARO_HZ) {
    float dt = (ahora_ms - ultimaLecturaBaro_ms) / 1000.0f;
    ultimaLecturaBaro_ms = ahora_ms;
    baro.leer(dt > 0.5f ? 0.05f : dt);
  }

  // ---------------------------------------------------- envíos a la web
  if (ahora_ms - ultimoEnvioWeb_ms >= PERIODO_WEB_MS) {          // 50 Hz orientación
    ultimoEnvioWeb_ms = ahora_ms;
    if (web::puedeEnviar()) enviarDatos();
  }
  if (ahora_ms - ultimoEnvioBaro_ms >= 1000 / ENVIO_BARO_HZ) {   // 10 Hz barómetro
    ultimoEnvioBaro_ms = ahora_ms;
    if (web::puedeEnviar()) enviarBarometro();
  }
  if (gps.posicionNueva() || ahora_ms - ultimoEnvioGps_ms >= 1000) {   // GPS
    ultimoEnvioGps_ms = ahora_ms;
    if (web::puedeEnviar()) enviarGps();
    gps.nmea.location.lat();   // leerla "consume" la novedad (isUpdated vuelve a false)
  }

  // ------------------------------------------------- 1 Hz: contabilidad
  if (ahora_ms - inicioSegundo_ms >= 1000) {
    frecuenciaReal_hz = muestrasEnEsteSegundo * 1000.0f / (ahora_ms - inicioSegundo_ms);
    muestrasEnEsteSegundo = 0;
    inicioSegundo_ms = ahora_ms;
    web::mantenimiento();
    informeSerie();
    erroresI2C = 0;
  }

  // ---------------------------------------- entradas: web, botón, serie
  atenderOrdenesWeb();
  atenderBoton();
  atenderSerie();
  digitalWrite(PIN_LED, imu.estado() == EstadoCalibracion::Midiendo);  // LED encendido = calibrando
}
