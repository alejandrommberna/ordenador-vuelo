// =============================================================================
//  madgwick.h — Fusión de giróscopo + acelerómetro (filtro de Madgwick)
// =============================================================================
//  Tenemos dos "fuentes" de orientación, cada una buena en una cosa:
//   · Giróscopo: rapidísimo y suave, pero solo da VELOCIDAD de giro; al
//     integrarla acumula error y la orientación deriva poco a poco.
//   · Acelerómetro: sabe dónde está "abajo" (la gravedad) y no deriva nunca,
//     pero es ruidoso y le afecta cualquier sacudida.
//  El filtro hace un "crossfade" permanente: sigue al giróscopo y, a cada
//  muestra, le da un empujoncito hacia lo que dice la gravedad. El tamaño
//  de ese empujón es beta.
//
//  La orientación se guarda como CUATERNIÓN (q0, q1, q2, q3) y no como
//  ángulos de Euler, porque los ángulos se vuelven locos al apuntar en
//  vertical (bloqueo de cardán)... justo donde apunta un cohete.
//
//  Convención: q gira vectores de los ejes del SENSOR a los ejes del MUNDO
//  (Z del mundo = hacia arriba). Sin magnetómetro, el giro alrededor de la
//  vertical (yaw) no tiene referencia absoluta: es relativo al arranque.
// =============================================================================
#pragma once

class Madgwick {
 public:
  float beta = 0.1f;                       // ganancia de corrección (rad/s)
  float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // orientación actual (empieza "sin girar")
  bool  usandoGravedad = true;             // ¿se usó el acelerómetro en el último paso?

  // Un paso del filtro.
  //   gx, gy, gz : velocidad angular en rad/s (¡ya calibrada!)
  //   ax, ay, az : aceleración en cualquier unidad (se normaliza)
  //   dt         : tiempo desde el paso anterior, en segundos
  //   tolerancia : si |a| se aleja de 1 g más que esto, no se usa la gravedad
  void actualizar(float gx, float gy, float gz,
                  float ax, float ay, float az, float dt, float tolerancia_g);

  // Pone la orientación directamente a partir de la gravedad (inclinación
  // exacta, yaw = 0). Útil al arrancar para no esperar a que converja.
  void alinear(float ax, float ay, float az);
};
