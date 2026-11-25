// =============================================================================
//  baro.h — Barómetro (BME280 o BMP280): presión -> altitud y velocidad vertical
// =============================================================================
//  El aire pesa: cuanto más subes, menos aire tienes encima y menos presión
//  mide el sensor. Cerca del suelo, 1 metro de subida son unos 12 Pa menos
//  (0,12 hPa). El BME280 resuelve unos pocos Pa, así que "ve" escalones de
//  10-20 cm: suficiente para notar que levantas la placa de la mesa.
// =============================================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>

class Barometro {
 public:
  // Busca el sensor en 0x76 y 0x77, averigua qué chip es y deja la altitud a 0.
  bool iniciar(TwoWire& bus);
  bool presente() const { return tipo_ != Ninguno; }
  const char* modelo() const;

  // Nueva lectura (llamar ~20 veces/s). dt = segundos desde la anterior.
  void leer(float dt);

  // La presión actual pasa a ser el "suelo": altitud = 0 y se borra la máxima.
  void ponerACero();

  // ---- Resultados
  float presion_hpa      = 0;    // presión (hPa = mbar)
  float temperatura_c    = 0;    // temperatura del barómetro
  float humedad          = NAN;  // % (solo el BME280; el BMP280 no mide humedad)
  float altitudIsa_m     = 0;    // altitud "estándar" (supone 1013,25 hPa a nivel del mar)
  float altitud_m        = 0;    // altitud sobre el punto de cero, filtrada
  float velocidadVert_ms = 0;    // velocidad vertical (+ sube, - baja)
  float altitudMax_m     = 0;    // la más alta desde el último cero (apogeo)

 private:
  enum Tipo { Ninguno, BME, BMP } tipo_ = Ninguno;
  float leerPresionPa();

  float presionSuelo_pa_  = 101325;  // presión de referencia: ahí la altitud es 0
};
