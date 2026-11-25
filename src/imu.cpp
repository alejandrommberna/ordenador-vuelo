// =============================================================================
//  imu.cpp — Lectura del MPU6050 y calibración del giróscopo
// =============================================================================
//  ¿Por qué hay que calibrar el giróscopo?
//  Aunque la placa esté completamente quieta, el MPU6050 nunca marca 0 °/s:
//  cada eje tiene un pequeño "offset" de fábrica (sesgo o bias), por ejemplo
//  -1,3 °/s en X. Es como un fader que no baja del todo a cero: parece poco,
//  pero el filtro INTEGRA (suma) esa velocidad continuamente, así que 1 °/s
//  de error son 60° de giro falso en un minuto.
//
//  La solución: con la placa quieta medimos ese offset (la media de muchas
//  muestras) y a partir de entonces se lo restamos a cada lectura.
//  Además medimos la desviación típica: si es grande, es que la placa se ha
//  movido durante la medida y la calibración no vale.
// =============================================================================
#include "imu.h"
#include "config.h"
#include <Preferences.h>   // memoria no volátil (NVS) del ESP32: sobrevive a apagados

// Registros del MPU6050 que usamos (mismo mapa en el MPU6500 y clones compatibles)
static constexpr uint8_t REG_DIVISOR    = 0x19;  // divisor de la frecuencia de muestreo
static constexpr uint8_t REG_CONFIG     = 0x1A;  // filtro paso bajo (DLPF)
static constexpr uint8_t REG_GIRO_CFG   = 0x1B;  // rango del giróscopo
static constexpr uint8_t REG_ACEL_CFG   = 0x1C;  // rango del acelerómetro
static constexpr uint8_t REG_DATOS      = 0x3B;  // primer byte de los 14 de datos
static constexpr uint8_t REG_ENERGIA    = 0x6B;  // gestión de energía (arranca dormido)
static constexpr uint8_t REG_QUIEN_SOY  = 0x75;  // WHO_AM_I: el "DNI" del chip

// --------------------------------------------------------- acceso al bus I2C
bool Imu::escribirRegistro(uint8_t registro, uint8_t valor) {
  bus_->beginTransmission(dir_);
  bus_->write(registro);
  bus_->write(valor);
  return bus_->endTransmission() == 0;      // 0 = el chip ha dicho "recibido" (ACK)
}

bool Imu::leerRegistros(uint8_t primero, uint8_t* destino, uint8_t cuantos) {
  // 1) le decimos al chip desde qué registro queremos leer...
  bus_->beginTransmission(dir_);
  bus_->write(primero);
  if (bus_->endTransmission(false) != 0) return false;   // false = sin STOP, seguimos hablando
  // 2) ...y le pedimos N bytes seguidos (el chip avanza de registro solo)
  if (bus_->requestFrom(dir_, cuantos) != cuantos) return false;
  for (uint8_t i = 0; i < cuantos; i++) destino[i] = bus_->read();
  return true;
}

// ----------------------------------------------------------------- iniciar
bool Imu::iniciar(TwoWire& bus, uint8_t direccion) {
  bus_ = &bus;
  dir_ = direccion;

  // ¿Quién eres? Un MPU6050 original contesta 0x68; los clones y "compatibles"
  // que venden como MPU6050 contestan otra cosa (0x70 = MPU6500, 0x71 = MPU9250...).
  // Funcionan igual con estos registros, así que solo avisamos.
  if (!leerRegistros(REG_QUIEN_SOY, &quienSoy_, 1)) {
    Serial.printf("  Nadie responde en 0x%02X (bus SDA %d / SCL %d)\n", dir_, PIN_SENS_SDA, PIN_SENS_SCL);
    return false;
  }
  const char* modelo = quienSoy_ == 0x68 ? "MPU6050 original"
                     : quienSoy_ == 0x70 ? "MPU6500 (compatible)"
                     : quienSoy_ == 0x71 ? "MPU9250 (compatible)"
                     : quienSoy_ == 0x73 ? "MPU9255 (compatible)" : "clon desconocido";
  Serial.printf("  WHO_AM_I = 0x%02X -> %s\n", quienSoy_, modelo);
  if (quienSoy_ == 0x00 || quienSoy_ == 0xFF) {
    Serial.println("  Ese valor parece ruido del bus, no un chip: revisa soldaduras y pull-ups");
    return false;
  }

  // Rango elegido en config.h -> valor del registro y "cuentas por unidad"
  // (el conversor da números de -32768 a 32767 para todo el rango)
  uint8_t regGiro, regAcel;
  switch (RANGO_GIRO_DPS) {
    case 250:  regGiro = 0x00; cuentasPorDps_ = 131.0f; break;
    case 500:  regGiro = 0x08; cuentasPorDps_ = 65.5f;  break;
    case 1000: regGiro = 0x10; cuentasPorDps_ = 32.8f;  break;
    default:   regGiro = 0x18; cuentasPorDps_ = 16.4f;  break;   // 2000
  }
  switch (RANGO_ACEL_G) {
    case 2:  regAcel = 0x00; cuentasPorG_ = 16384; break;
    case 4:  regAcel = 0x08; cuentasPorG_ = 8192;  break;
    case 8:  regAcel = 0x10; cuentasPorG_ = 4096;  break;
    default: regAcel = 0x18; cuentasPorG_ = 2048;  break;          // 16
  }

  escribirRegistro(REG_ENERGIA, 0x80);   // reset completo: volver a valores de fábrica
  delay(100);
  escribirRegistro(REG_ENERGIA, 0x01);   // despertar, con el reloj del giróscopo X (más estable)
  delay(10);
  escribirRegistro(REG_CONFIG, FILTRO_DLPF);   // filtro paso bajo ~44 Hz
  escribirRegistro(REG_DIVISOR, 4);            // 1 kHz / (1 + 4) = 200 Hz, igual que nuestro bucle
  escribirRegistro(REG_GIRO_CFG, regGiro);
  escribirRegistro(REG_ACEL_CFG, regAcel);

  // Comprobación: leemos lo que acabamos de escribir. Si no coincide, algo falla en el bus.
  uint8_t leido[2];
  if (!leerRegistros(REG_GIRO_CFG, leido, 2) || leido[0] != regGiro || leido[1] != regAcel) {
    Serial.println("  El chip no ha guardado la configuración: bus I2C inestable");
    return false;
  }

  cargarDeFlash();  // ¿hay una calibración de otra vez?
  return true;
}

// -------------------------------------------------------------------- leer
bool Imu::leer() {
  // Los 14 bytes de golpe, empezando en 0x3B:
  //   acel X, Y, Z | temperatura | giro X, Y, Z   (cada valor: 2 bytes, el alto primero)
  // Leerlos en una sola ráfaga garantiza que los 7 valores son del mismo instante.
  uint8_t b[14];
  if (!leerRegistros(REG_DATOS, b, 14)) return false;

  int16_t v[7];
  for (int i = 0; i < 7; i++) v[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);

  // Cuentas del conversor -> unidades que se entienden (g y °/s)
  for (int i = 0; i < 3; i++) {
    acel_g[i]        = v[i] / cuentasPorG_;
    giroCrudo_dps[i] = v[4 + i] / cuentasPorDps_;
  }
  // Fórmula de temperatura del datasheet (distinta en la familia MPU6500)
  temperatura_c = quienSoy_ == 0x68 ? v[3] / 340.0f + 36.53f : v[3] / 333.87f + 21.0f;

  // LA CORRECCIÓN: lectura - sesgo. Esto es literalmente toda la calibración
  // en uso; lo difícil es medir bien el sesgo.
  for (int i = 0; i < 3; i++) giro_dps[i] = giroCrudo_dps[i] - sesgo_dps_[i];

  if (estado_ == EstadoCalibracion::Midiendo) acumularMuestraCalibracion();
  return true;
}

// ------------------------------------------------------------- calibración
void Imu::empezarCalibracion() {
  contador_ = 0;
  sumaTemp_ = 0;
  for (int i = 0; i < 3; i++) suma_[i] = sumaCuadrados_[i] = 0;
  estado_ = EstadoCalibracion::Midiendo;
  snprintf(mensaje_, sizeof(mensaje_), "Midiendo... no muevas la placa");
  Serial.println("[CAL] Calibrando el giróscopo: deja la placa QUIETA 3 s");
}

void Imu::acumularMuestraCalibracion() {
  contador_++;
  // Las primeras muestras se tiran: al pulsar el botón (o la pantalla del
  // móvil apoyado en la mesa) la placa todavía está temblando.
  if (contador_ <= MUESTRAS_ASENTAMIENTO) return;

  // Usamos la lectura CRUDA: queremos medir el offset real del sensor, no
  // lo que queda después de restarle la calibración anterior.
  for (int i = 0; i < 3; i++) {
    suma_[i]          += giroCrudo_dps[i];
    sumaCuadrados_[i] += (double)giroCrudo_dps[i] * giroCrudo_dps[i];
  }
  sumaTemp_ += temperatura_c;

  if (contador_ >= MUESTRAS_ASENTAMIENTO + MUESTRAS_CALIBRACION) terminarCalibracion();
}

void Imu::terminarCalibracion() {
  const double n = MUESTRAS_CALIBRACION;
  float media[3], desv[3];
  float peorDesv = 0, peorSesgo = 0;

  for (int i = 0; i < 3; i++) {
    // Media = el sesgo que buscamos
    media[i] = suma_[i] / n;
    // Varianza = media de los cuadrados - cuadrado de la media
    double varianza = sumaCuadrados_[i] / n - (double)media[i] * media[i];
    if (varianza < 0) varianza = 0;     // por si el redondeo da -0,0000001
    desv[i] = sqrt(varianza);           // desviación típica = "cuánto tiembla"
    peorDesv  = max(peorDesv, desv[i]);
    peorSesgo = max(peorSesgo, fabsf(media[i]));
  }
  for (int i = 0; i < 3; i++) desviacion_dps_[i] = desv[i];

  novedad_ = true;

  // ¿Se ha movido durante la medida?
  if (peorDesv > UMBRAL_QUIETUD_DPS) {
    estado_ = EstadoCalibracion::Fallida;
    snprintf(mensaje_, sizeof(mensaje_), "Se ha movido (ruido %.2f °/s > %.2f). %s",
             peorDesv, UMBRAL_QUIETUD_DPS,
             tieneCalibracion_ ? "Sigo con la anterior" : "Repite quieta");
    Serial.printf("[CAL] FALLO: %s\n", mensaje_);
    return;  // no tocamos el sesgo en uso
  }
  // ¿El sesgo es razonable?
  if (peorSesgo > SESGO_MAXIMO_DPS) {
    estado_ = EstadoCalibracion::Fallida;
    snprintf(mensaje_, sizeof(mensaje_), "Sesgo anormal (%.1f °/s). ¿Sensor dañado o girando?", peorSesgo);
    Serial.printf("[CAL] FALLO: %s\n", mensaje_);
    return;
  }

  // Todo bien: este es el nuevo sesgo
  for (int i = 0; i < 3; i++) sesgo_dps_[i] = media[i];
  tempCalibracion_c_ = sumaTemp_ / n;
  tieneCalibracion_ = true;
  estado_ = EstadoCalibracion::Correcta;
  snprintf(mensaje_, sizeof(mensaje_), "Calibrado a %.1f °C", tempCalibracion_c_);
  guardarEnFlash();

  Serial.printf("[CAL] OK  sesgo = (%+.3f, %+.3f, %+.3f) °/s  ruido = (%.3f, %.3f, %.3f) °/s  T = %.1f °C\n",
                sesgo_dps_[0], sesgo_dps_[1], sesgo_dps_[2],
                desv[0], desv[1], desv[2], tempCalibracion_c_);
}

void Imu::borrarCalibracion() {
  for (int i = 0; i < 3; i++) sesgo_dps_[i] = desviacion_dps_[i] = 0;
  tieneCalibracion_ = false;
  estado_ = EstadoCalibracion::SinCalibrar;
  snprintf(mensaje_, sizeof(mensaje_), "Calibración borrada");
  Preferences p;
  p.begin("imu", false);
  p.clear();
  p.end();
  novedad_ = true;
  Serial.println("[CAL] Calibración borrada");
}

float Imu::progreso() const {
  if (estado_ != EstadoCalibracion::Midiendo) return -1;
  return (float)contador_ / (MUESTRAS_ASENTAMIENTO + MUESTRAS_CALIBRACION);
}

bool Imu::hayNovedad() {
  bool n = novedad_;
  novedad_ = false;
  return n;
}

// ----------------------------------------------------- memoria no volátil
// Preferences guarda pares clave-valor en la flash del ESP32 (como una
// memoria de escenas de la mesa: apagas y sigue ahí).
void Imu::guardarEnFlash() {
  Preferences p;
  p.begin("imu", false);              // "imu" = espacio de nombres; false = lectura y escritura
  p.putFloat("bx", sesgo_dps_[0]);
  p.putFloat("by", sesgo_dps_[1]);
  p.putFloat("bz", sesgo_dps_[2]);
  p.putFloat("sx", desviacion_dps_[0]);
  p.putFloat("sy", desviacion_dps_[1]);
  p.putFloat("sz", desviacion_dps_[2]);
  p.putFloat("temp", tempCalibracion_c_);
  p.putBool("ok", true);
  p.end();
}

void Imu::cargarDeFlash() {
  Preferences p;
  p.begin("imu", false);              // (en solo lectura daría un aviso feo la 1.ª vez)
  tieneCalibracion_ = p.getBool("ok", false);
  if (tieneCalibracion_) {
    sesgo_dps_[0] = p.getFloat("bx", 0);
    sesgo_dps_[1] = p.getFloat("by", 0);
    sesgo_dps_[2] = p.getFloat("bz", 0);
    desviacion_dps_[0] = p.getFloat("sx", 0);
    desviacion_dps_[1] = p.getFloat("sy", 0);
    desviacion_dps_[2] = p.getFloat("sz", 0);
    tempCalibracion_c_ = p.getFloat("temp", 0);
    estado_ = EstadoCalibracion::Correcta;
    snprintf(mensaje_, sizeof(mensaje_), "Cargada de memoria (%.1f °C)", tempCalibracion_c_);
    Serial.printf("  Calibración cargada: sesgo = (%+.3f, %+.3f, %+.3f) °/s a %.1f °C\n",
                  sesgo_dps_[0], sesgo_dps_[1], sesgo_dps_[2], tempCalibracion_c_);
  } else {
    Serial.println("  No hay calibración guardada");
  }
  p.end();
}
