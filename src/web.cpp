// =============================================================================
//  web.cpp — Red WiFi propia + servidor web + WebSocket
// =============================================================================
#include "web.h"
#include "config.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>

// La página web/index.html la mete PlatformIO dentro del firmware al compilar
// (opción board_build.embed_txtfiles del platformio.ini). Aquí solo
// declaramos el nombre con el que aparece en memoria: un texto terminado en 0.
extern const char PAGINA_HTML[] asm("_binary_web_index_html_start");

namespace web {

static AsyncWebServer servidor(80);   // puerto 80 = HTTP normal
static AsyncWebSocket ws("/ws");      // el WebSocket vive en http://.../ws

// El servidor "asíncrono" atiende la red en su PROPIA tarea, en paralelo con
// loop(). Si desde ahí tocáramos directamente la IMU o el filtro, las dos
// tareas podrían pisarse. Por eso las órdenes se dejan en un buzón (una cola
// de FreeRTOS) y loop() las recoge cuando le toca: como pasar una nota por
// intercom al operador en vez de meter la mano en su mesa.
static QueueHandle_t buzon = nullptr;
static volatile bool clienteNuevo = false;

// Se ejecuta cada vez que pasa algo en el WebSocket
static void alEventoWebSocket(AsyncWebSocket* servidorWs, AsyncWebSocketClient* cliente,
                              AwsEventType tipo, void* arg, uint8_t* datos, size_t longitud) {
  switch (tipo) {
    case WS_EVT_CONNECT:
      Serial.printf("[WEB] Cliente #%u conectado desde %s\n", (unsigned)cliente->id(),
                    cliente->remoteIP().toString().c_str());
      clienteNuevo = true;
      break;

    case WS_EVT_DISCONNECT:
      Serial.printf("[WEB] Cliente #%u desconectado\n", (unsigned)cliente->id());
      break;

    case WS_EVT_DATA: {
      // Solo aceptamos mensajes de texto cortos que lleguen en un único trozo
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (!(info->final && info->index == 0 && info->len == longitud && info->opcode == WS_TEXT)) break;

      char texto[32];
      size_t n = min(longitud, sizeof(texto) - 1);
      memcpy(texto, datos, n);
      texto[n] = '\0';

      Mensaje m = {Orden::Calibrar, 0};
      if      (strcmp(texto, "cal") == 0)       m.orden = Orden::Calibrar;
      else if (strcmp(texto, "borrar") == 0)    m.orden = Orden::Borrar;
      else if (strcmp(texto, "realinear") == 0) m.orden = Orden::Realinear;
      else if (strcmp(texto, "cero") == 0)      m.orden = Orden::CeroAltitud;
      else if (strncmp(texto, "beta=", 5) == 0) { m.orden = Orden::CambiarBeta; m.valor = atof(texto + 5); }
      else {
        Serial.printf("[WEB] Orden desconocida: %s\n", texto);
        break;
      }
      xQueueSend(buzon, &m, 0);   // 0 = si el buzón está lleno, no esperar
      break;
    }

    default:
      break;
  }
}

void iniciar() {
  buzon = xQueueCreate(8, sizeof(Mensaje));

  // 1) La placa crea su propia red WiFi (modo "punto de acceso").
  //    No depende de ningún router: funciona igual en el campo.
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_NOMBRE, WIFI_CLAVE);
  Serial.printf("  WiFi \"%s\" (clave \"%s\")  ->  http://%s\n",
                WIFI_NOMBRE, WIFI_CLAVE, WiFi.softAPIP().toString().c_str());

  // 2) mDNS: permite escribir http://vuelo.local en vez de la IP
  if (MDNS.begin(NOMBRE_MDNS)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("  También en http://%s.local\n", NOMBRE_MDNS);
  }

  // 3) Rutas del servidor
  ws.onEvent(alEventoWebSocket);
  servidor.addHandler(&ws);

  servidor.on("/", HTTP_GET, [](AsyncWebServerRequest* peticion) {
    // Enviamos la página directamente desde la flash, sin copiarla a RAM
    peticion->send(200, "text/html; charset=utf-8",
                   (const uint8_t*)PAGINA_HTML, strlen(PAGINA_HTML));
  });

  // Cualquier otra dirección -> a la página principal
  servidor.onNotFound([](AsyncWebServerRequest* peticion) { peticion->redirect("/"); });

  servidor.begin();
}

bool puedeEnviar() {
  // availableForWriteAll(): si el navegador va lento (WiFi mala), su cola se
  // llena. En ese caso nos saltamos este envío en vez de acumular retraso:
  // para ver la orientación en directo, mejor un dato perdido que uno viejo.
  return ws.count() > 0 && ws.availableForWriteAll();
}

void enviarATodos(const char* texto) {
  if (ws.count() > 0) ws.textAll(texto);
}

bool siguienteOrden(Mensaje& m) {
  return buzon && xQueueReceive(buzon, &m, 0) == pdTRUE;
}

bool hayClienteNuevo() {
  bool n = clienteNuevo;
  clienteNuevo = false;
  return n;
}

void mantenimiento() {
  ws.cleanupClients();   // cierra conexiones de navegadores que ya no están
}

}  // namespace web
