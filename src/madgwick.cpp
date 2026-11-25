// =============================================================================
//  madgwick.cpp — Filtro de Madgwick (versión IMU: giróscopo + acelerómetro)
// =============================================================================
//  Basado en S. Madgwick, "An efficient orientation filter for inertial and
//  inertial/magnetic sensor arrays" (2010). Escrito de forma legible en vez
//  de la versión "optimizada" que circula por internet, para que se vea qué
//  hace cada parte. Las ecuaciones citadas son las del artículo.
// =============================================================================
#include "madgwick.h"
#include <math.h>

void Madgwick::actualizar(float gx, float gy, float gz,
                          float ax, float ay, float az, float dt, float tolerancia_g) {
  // ---------------------------------------------------------------------
  // 1) PREDICCIÓN CON EL GIRÓSCOPO  (ec. 11-12)
  //    Derivada del cuaternión = ½ · q ⊗ (0, ω)
  //    Es "si giro a esta velocidad durante dt, ¿dónde acabo?".
  // ---------------------------------------------------------------------
  float dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float dq1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  float dq2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  float dq3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  // ---------------------------------------------------------------------
  // 2) CORRECCIÓN CON LA GRAVEDAD  (ec. 25-26 y 44)
  //    Solo si el acelerómetro está midiendo (casi) solo la gravedad.
  //    Con el motor encendido mide empuje + gravedad: si lo usáramos,
  //    el filtro "creería" que abajo está en otro sitio.
  // ---------------------------------------------------------------------
  float modulo = sqrtf(ax * ax + ay * ay + az * az);   // |a| en g
  usandoGravedad = modulo > 0.0f && fabsf(modulo - 1.0f) < tolerancia_g;

  if (usandoGravedad && beta > 0.0f) {
    // Normalizamos: solo nos interesa la DIRECCIÓN de la gravedad
    ax /= modulo;  ay /= modulo;  az /= modulo;

    // f = (dónde DEBERÍA estar "arriba" según q) - (dónde lo MIDE el sensor)
    //     Las tres primeras componentes son la 3.ª fila de la matriz de
    //     rotación: el eje Z del mundo visto desde los ejes del sensor.
    float fx = 2.0f * (q1 * q3 - q0 * q2)        - ax;
    float fy = 2.0f * (q0 * q1 + q2 * q3)        - ay;
    float fz = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;

    // Gradiente = Jᵀ · f  (J = cómo cambia f al cambiar cada componente de q).
    // Apunta hacia donde AUMENTA el error, así que luego lo restamos.
    float s0 = -2.0f * q2 * fx + 2.0f * q1 * fy;
    float s1 =  2.0f * q3 * fx + 2.0f * q0 * fy - 4.0f * q1 * fz;
    float s2 = -2.0f * q0 * fx + 2.0f * q3 * fy - 4.0f * q2 * fz;
    float s3 =  2.0f * q1 * fx + 2.0f * q2 * fy;

    // Normalizamos el gradiente: así beta es directamente la "velocidad"
    // máxima de corrección, independiente de lo grande que sea el error.
    float norma = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (norma > 1e-9f) {
      dq0 -= beta * s0 / norma;
      dq1 -= beta * s1 / norma;
      dq2 -= beta * s2 / norma;
      dq3 -= beta * s3 / norma;
    }
  }

  // ---------------------------------------------------------------------
  // 3) INTEGRACIÓN  (ec. 42): q nuevo = q + dq · dt
  // ---------------------------------------------------------------------
  q0 += dq0 * dt;
  q1 += dq1 * dt;
  q2 += dq2 * dt;
  q3 += dq3 * dt;

  // Un cuaternión de rotación debe medir 1. Al sumar se "estira" un poco,
  // así que lo volvemos a normalizar en cada paso.
  float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= n;  q1 /= n;  q2 /= n;  q3 /= n;
}

void Madgwick::alinear(float ax, float ay, float az) {
  // Alabeo (roll, giro en X) y cabeceo (pitch, giro en Y) salen de dónde
  // cae la gravedad sobre los ejes del sensor. El yaw no se puede saber
  // sin brújula: lo ponemos a 0.
  float roll  = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

  // Ángulos de Euler (orden Z-Y-X) -> cuaternión, con yaw = 0
  float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
  q0 =  cr * cp;
  q1 =  sr * cp;
  q2 =  cr * sp;
  q3 = -sr * sp;
}
