# TODO-list


2. **Web data dashboard**
    * separate from the REPL
    * **Telemetry service** that collects sensor data, serializes to JSON and pushes over a WebSocket endpoint.
    * **Static dashboard** (HTML/JS/CSS + Chart.js or something similar) served by the same HTTP server, connects to `/ws/telemtry` for live charts.
    * PSRAM used for all the bulkier web-server buffers, since it doesn't need to be as fast.
4. Write our own NMEA parser to be able to ditch TinyGps++-espidf


## Feature-Based Architecture

Organize your firmware by self-contained features. Each feature folder contains everything needed for development, testing, and maintenance.

```
/components
├─ /dashboard
│  ├─ telemetry.cpp/.h          // Telemetry service: collects & queues JSON data
│  ├─ http_server.cpp/.h        // HTTP server: serves static files & WS endpoints
│  ├─ ws_telemetry.cpp/.h       // WebSocket endpoint for live telemetry
│  └─ static/                   // Web assets (HTML, JS, CSS, Chart.js)
│
└─ /config
   └─ json_config.cpp/.h        // SPIFFS/LittleFS mount + config.json read/write (cJSON wrapper)

```

### Namespaces

* `console::`
  Contains all REPL-related code (engine & transports).

* `dashboard::`
  Handles data-logging UI: telemetry gathering + web server.

* `config::`
  Manages persistent configuration via `config.json` on SPIFFS/LittleFS.
