// =============================================================================
//  web.h — Red WiFi propia + servidor web + WebSocket
// =============================================================================
//  · Servidor web: entrega la página (web/index.html, metida dentro del
//    firmware al compilar) cuando abres http://192.168.4.1
//  · WebSocket: una conexión que se queda ABIERTA entre la placa y el
//    navegador, por la que mandamos la orientación 50 veces por segundo sin
//    tener que pedirla cada vez (como un cable DMX siempre enviando, frente
//    a ir preguntando "¿y ahora?").
// =============================================================================
#pragma once
#include <Arduino.h>

namespace web {

// Órdenes que pueden llegar desde la página
enum class Orden : uint8_t { Calibrar, Borrar, Realinear, CambiarBeta, CeroAltitud };

struct Mensaje {
  Orden orden;
  float valor;  // solo se usa en CambiarBeta
};

void iniciar();                      // crea la red WiFi y arranca el servidor
bool puedeEnviar();                  // hay alguien conectado y no está saturado
void enviarATodos(const char* texto);
bool siguienteOrden(Mensaje& m);     // saca una orden del buzón (false si no hay)
bool hayClienteNuevo();              // true una vez por cada conexión nueva
void mantenimiento();                // limpieza periódica de conexiones muertas

}  // namespace web
