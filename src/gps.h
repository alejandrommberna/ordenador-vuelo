// =============================================================================
//  gps.h — GPS por UART (frases NMEA) con TinyGPSPlus
// =============================================================================
//  El GPS no hay que "preguntarle": habla solo, sin parar, mandando frases de
//  texto (NMEA) por el puerto serie, como un ticker de noticias:
//    $GNGGA,185502.00,4024.91,N,00341.06,W,1,09,1.02,655.3,M,...
//  TinyGPSPlus lee esas frases letra a letra y nos da los números ya
//  separados (latitud, satélites, altitud...). Nuestro trabajo es solo ir
//  vaciando el buzón del puerto serie a menudo para que no se desborde.
// =============================================================================
#pragma once
#include <Arduino.h>
#include <TinyGPSPlus.h>

class Gps {
 public:
  void iniciar();
  void atender();               // lee todo lo que haya llegado (llamar en cada loop)

  bool mudo() const;            // más de 2 s sin recibir nada -> cable, alimentación o baudios
  uint8_t tipoFix() const;      // 0 = sin posición, 2 = 2D (sin altitud), 3 = 3D
  bool posicionNueva();         // true una vez por cada posición nueva

  TinyGPSPlus nmea;             // el intérprete; sus datos se leen directamente

 private:
  uint32_t ultimoByte_ms_ = 0;
};
