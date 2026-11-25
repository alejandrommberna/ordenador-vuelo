// =============================================================================
//  gps.cpp — GPS por UART (frases NMEA) con TinyGPSPlus
// =============================================================================
#include "gps.h"
#include "config.h"

void Gps::iniciar() {
  // Buzón de recepción más grande (por defecto 256 bytes): así no se pierde
  // nada si el loop se entretiene un momento (p. ej. al realinear la IMU).
  Serial1.setRxBufferSize(1024);
  Serial1.begin(GPS_BAUDIOS, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.printf("  GPS escuchando en GPIO%d a %d baudios (primer fix: 1-5 min con cielo despejado)\n",
                PIN_GPS_RX, GPS_BAUDIOS);
}

void Gps::atender() {
  while (Serial1.available()) {
    nmea.encode(Serial1.read());   // cada carácter al intérprete
    ultimoByte_ms_ = millis();
  }
}

bool Gps::mudo() const {
  return ultimoByte_ms_ == 0 || millis() - ultimoByte_ms_ > 2000;
}

uint8_t Gps::tipoFix() const {
  // TinyGPSPlus no da el tipo de fix directamente: lo deducimos.
  // Posición reciente -> al menos 2D; si además hay altitud reciente -> 3D.
  if (!nmea.location.isValid() || nmea.location.age() > 3000) return 0;
  if (nmea.altitude.isValid() && nmea.altitude.age() < 3000) return 3;
  return 2;
}

bool Gps::posicionNueva() {
  return nmea.location.isUpdated();   // se "consume" al leer lat()/lng()
}
