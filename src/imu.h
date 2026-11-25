// =============================================================================
//  imu.h — Lectura del MPU6050 y calibración del giróscopo
// =============================================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>

// En qué punto está la calibración
enum class EstadoCalibracion : uint8_t {
  SinCalibrar,   // nunca se ha calibrado (no hay nada guardado)
  Midiendo,      // promediando muestras ahora mismo
  Correcta,      // hay una calibración válida en uso
  Fallida        // el último intento falló (se sigue usando la anterior, si había)
};

class Imu {
 public:
  // Arranca el sensor y carga la calibración guardada en la memoria flash.
  bool iniciar(TwoWire& bus, uint8_t direccion);

  // Lee una muestra nueva del sensor y la deja en las variables de abajo.
  // Devuelve false si falla el I2C (cable flojo, interferencia...).
  bool leer();

  uint8_t quienSoy() const { return quienSoy_; }   // registro WHO_AM_I del chip

  // ---- Última muestra leída (unidades "humanas")
  float giro_dps[3]      = {0, 0, 0};  // velocidad angular YA corregida (°/s)
  float giroCrudo_dps[3] = {0, 0, 0};  // tal cual sale del sensor (°/s)
  float acel_g[3]        = {0, 0, 1};  // aceleración (en g, 1 g = 9,81 m/s²)
  float temperatura_c    = 0;          // temperatura del chip (°C)

  // ---- Calibración
  void empezarCalibracion();
  void borrarCalibracion();
  EstadoCalibracion estado() const { return estado_; }
  bool tieneCalibracion() const { return tieneCalibracion_; }
  float progreso() const;                       // 0..1 midiendo; -1 si no
  const float* sesgo() const { return sesgo_dps_; }          // lo que se resta
  const float* desviacion() const { return desviacion_dps_; } // ruido medido
  float temperaturaCalibracion() const { return tempCalibracion_c_; }
  const char* mensaje() const { return mensaje_; }

  // Devuelve true UNA sola vez cuando la calibración acaba (bien o mal),
  // para que el programa principal reaccione (avisar a la web, realinear...).
  bool hayNovedad();

 private:
  bool escribirRegistro(uint8_t registro, uint8_t valor);
  bool leerRegistros(uint8_t primero, uint8_t* destino, uint8_t cuantos);
  void acumularMuestraCalibracion();
  void terminarCalibracion();
  void guardarEnFlash();
  void cargarDeFlash();

  TwoWire* bus_ = nullptr;
  uint8_t dir_ = 0x68;
  uint8_t quienSoy_ = 0;
  float cuentasPorDps_ = 16.4f;   // cuántas cuentas del conversor son 1 °/s
  float cuentasPorG_   = 2048;    // cuántas cuentas son 1 g

  EstadoCalibracion estado_ = EstadoCalibracion::SinCalibrar;
  bool tieneCalibracion_ = false;
  bool novedad_ = false;

  float sesgo_dps_[3]      = {0, 0, 0};
  float desviacion_dps_[3] = {0, 0, 0};
  float tempCalibracion_c_ = 0;
  char  mensaje_[96]       = "Sin calibrar";

  // Acumuladores de la medida en curso (double para no perder precisión al
  // sumar cientos de valores pequeños).
  uint16_t contador_ = 0;
  double suma_[3] = {0, 0, 0};
  double sumaCuadrados_[3] = {0, 0, 0};
  double sumaTemp_ = 0;
};
