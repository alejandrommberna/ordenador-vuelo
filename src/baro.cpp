// =============================================================================
//  baro.cpp — Barómetro (BME280 o BMP280)
// =============================================================================
#include "baro.h"
#include "config.h"
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>

static Adafruit_BME280 bme;
static Adafruit_BMP280* bmp = nullptr;   // se crea al saber en qué bus está

// Pregunta al chip quién es (registro 0xD0), como en tu transmisor:
// 0x60 = BME280 (presión + temperatura + humedad), 0x58 = BMP280 (sin humedad).
// Por fuera las placas "GY-BME/PMP280" son iguales, así que hay que preguntar.
static int leerIdChip(TwoWire& bus, uint8_t dir) {
  bus.beginTransmission(dir);
  bus.write(0xD0);
  if (bus.endTransmission(false) != 0) return -1;      // nadie contesta en esa dirección
  if (bus.requestFrom(dir, (uint8_t)1) != 1) return -1;
  return bus.read();
}

// Fórmula barométrica de la atmósfera estándar: altitud a partir del
// cociente entre la presión actual y la de referencia.
static float altitudDesdePresion(float p, float pReferencia) {
  return 44330.0f * (1.0f - powf(p / pReferencia, 0.190295f));
}

bool Barometro::iniciar(TwoWire& bus) {
  const uint8_t direcciones[] = {0x76, 0x77};   // SDO a GND -> 0x76, SDO a 3V3 -> 0x77
  for (uint8_t d : direcciones) {
    int id = leerIdChip(bus, d);
    if (id < 0) continue;
    Serial.printf("  Barómetro en 0x%02X, chip ID 0x%02X -> ", d, id);

    // Configuración para medir altitud (no el tiempo que hace):
    //  · sobremuestreo x16 en presión: el chip promedia 16 medidas internas
    //  · filtro IIR x4: suaviza sin meter demasiado retraso
    //  · modo normal sin pausa: mide continuamente (~23 veces por segundo)
    if (id == 0x60 && bme.begin(d, &bus)) {
      bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                      Adafruit_BME280::SAMPLING_X2,     // temperatura
                      Adafruit_BME280::SAMPLING_X16,    // presión
                      Adafruit_BME280::SAMPLING_X1,     // humedad
                      Adafruit_BME280::FILTER_X4,
                      Adafruit_BME280::STANDBY_MS_0_5);
      tipo_ = BME;
    } else if (id == 0x58) {
      bmp = new Adafruit_BMP280(&bus);
      if (bmp->begin(d, 0x58)) {
        bmp->setSampling(Adafruit_BMP280::MODE_NORMAL,
                         Adafruit_BMP280::SAMPLING_X2,
                         Adafruit_BMP280::SAMPLING_X16,
                         Adafruit_BMP280::FILTER_X4,
                         Adafruit_BMP280::STANDBY_MS_1);
        tipo_ = BMP;
      }
    }
    if (tipo_ == Ninguno) { Serial.println("chip desconocido"); continue; }
    Serial.println(modelo());

    // Dejamos que el filtro del chip se asiente y promediamos 1 s de lecturas:
    // esa presión será el "suelo" (altitud 0)
    delay(200);
    float suma = 0;
    for (int i = 0; i < 20; i++) { suma += leerPresionPa(); delay(50); }
    presionSuelo_pa_ = suma / 20;
    altitud_m = velocidadVert_ms = altitudMax_m = 0;
    Serial.printf("  Cero de altitud en %.2f hPa\n", presionSuelo_pa_ / 100.0f);
    return true;
  }
  Serial.printf("  Barómetro NO encontrado en 0x76/0x77 (bus SDA %d / SCL %d)\n", PIN_SENS_SDA, PIN_SENS_SCL);
  return false;
}

const char* Barometro::modelo() const {
  return tipo_ == BME ? "BME280" : tipo_ == BMP ? "BMP280" : "ninguno";
}

float Barometro::leerPresionPa() {
  return tipo_ == BME ? bme.readPressure() : bmp->readPressure();
}

void Barometro::ponerACero() {
  // En vez de usar una lectura suelta (ruidosa), movemos la referencia a la
  // presión que corresponde a la altitud YA filtrada: así el cero es limpio
  // aunque la placa se esté moviendo. Es la fórmula barométrica al revés.
  presionSuelo_pa_ *= powf(1.0f - altitud_m / 44330.0f, 5.255f);
  altitud_m = 0;
  altitudMax_m = 0;
  Serial.printf("[BARO] Cero de altitud en %.2f hPa\n", presionSuelo_pa_ / 100.0f);
}

void Barometro::leer(float dt) {
  if (!presente() || dt <= 0) return;

  float p = leerPresionPa();
  if (!(p > 30000 && p < 110000)) return;   // lectura absurda (fallo de I2C): la ignoramos

  if (tipo_ == BME) {
    temperatura_c = bme.readTemperature();
    humedad = bme.readHumidity();
  } else {
    temperatura_c = bmp->readTemperature();
  }
  presion_hpa = p / 100.0f;
  altitudIsa_m = altitudDesdePresion(p, 101325.0f);
  float medida = altitudDesdePresion(p, presionSuelo_pa_);

  // ---------------------------------------------------------------------
  // FILTRO ALFA-BETA: altitud y velocidad vertical a partir de la medida.
  // Derivar directamente la altitud ruidosa daría una velocidad que salta
  // como loca. En su lugar:
  //   1. PREDECIR: si iba a v m/s, ahora estaré en h + v·dt
  //   2. COMPARAR: la diferencia entre lo medido y lo predicho (el "residuo")
  //   3. CORREGIR: un poco la altitud (ALFA) y un poco la velocidad (BETA)
  // Es como un seguidor con un operador que anticipa el movimiento del
  // artista y corrige suavemente cuando se desvía, en vez de dar tirones.
  // ---------------------------------------------------------------------
  const float ALFA = 0.25f, BETA = 0.03f;
  altitud_m += velocidadVert_ms * dt;          // 1. predicción
  float residuo = medida - altitud_m;          // 2. comparación
  altitud_m += ALFA * residuo;                 // 3. corrección
  velocidadVert_ms += (BETA / dt) * residuo;

  if (altitud_m > altitudMax_m) altitudMax_m = altitud_m;
}
