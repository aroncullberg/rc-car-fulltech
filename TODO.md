# TODO-list


1. **Unified REPL console**
    * One esp\_console-based engine, accessible over:
        * USB-OTG (USB-CDC virtual COM)
        * Telnet (TCP on port 23)
        * in-browser (Websocket -> something)
    * All three transmorts feed into the same command parser and share stdout
2. **Web data dashboard**
    * separate from the REPL
    * **Telemetry service** that collects sensor data, serializes to JSON and pushes over a WebSocket endpoint.
    * **Static dashboard** (HTML/JS/CSS + Chart.js or something similar) served by the same HTTP server, connects to `/ws/telemtry` for live charts.
    * PSRAM used for all the bulkier web-server buffers, since it doesn't need to be as fast.
3. Persistent configuration
    * Mount a tiny filesystem (SPIFFS or LittleFS) on flash
    * store `config.json`, parsed/updated with JSON parser of choice.
        * Arbitrary key names (no 15-char limit)
        * easy listing and human readable format
        * learn more about 'wear-leveling' in Flash FS
    * On config "commit", overwrite `config.json` ; in-ram changes only applied when you write back.
4. Write our own NMEA parser to be able to ditch TinyGps++-espidf

## implementation order (preliminary)
1. Console (USB-OTG only)
    * Implement `console_engine.cpp` (esp_console + linenoise). 
    * Hook up `usb_transport.cpp` so that plugs-in USB→OTG and you get a working REPL on your PC’s COM port. 
    * ✅ Smoke-test: power-cycle, type commands, see output.

2. Config (SPIFFS + config.json)
   * In `json_config.cpp`, mount SPIFFS/LittleFS (in Flash or PSRAM).
   * Use cJSON (or similar) to load/save an in-RAM JSON object on boot/commit.
   * Expose `config get/set/commit` commands in your USB REPL.
   * ✅ Smoke-test: change a value, `commit`, reboot, `get `again.

3. Console (network transports)
   * Add `telnet_transport.cpp`: spin up a TCP listener and VFS→esp_console integration.
   * Add `ws_transport.cpp`: WebSocket bridge for in-browser REPL (xterm.js client later).
   * ✅ Smoke-test: connect via telnet client and via browser, run the same commands.

4. Dashboard – Telemetry Service
    * In `telemetry.cpp`, gather your sensor/FSM state, serialize to JSON, push into a FreeRTOS queue.
    * Expose an API in code to “start/stop” telemetry for later UI control.
    * ✅ Smoke-test: call the service from REPL (`startTelemetry`), see it enqueue data.

5. Dashboard – HTTP + WS Endpoints
    * In `http_server.cpp`, bring up the HTTP server on your AP.
    * Mount SPIFFS so you can GET `/static/index.html`, `/static/app.js`, etc.
    * In `ws_telemetry.cpp`, implement `/ws/telemetry` that reads JSON from your telemetry queue and streams it.

6. Dashboard – Static Web UI
    * Under `static/`, build your HTML/CSS/JS:
        * Load Chart.js to plot incoming telemetry.
        * Load xterm.js (or similar) to connect to `/ws/console`.
    * ✅ Smoke-test: open `http://<esp-ip>/`, see live charts plus embedded console.

7. Integration & Polish
    * Wire config knobs (e.g. telemetry interval) into your REPL and dashboard.
    * Tune PSRAM allocations for server buffers.
    * Final end-to-end tests: power-cycle, AP up, REPL on all three transports, dashboard live.



## Feature-Based Architecture

Organize your firmware by self-contained features. Each feature folder contains everything needed for development, testing, and maintenance.

```
/components
├─ /console
│  ├─ console_engine.cpp/.h     // Core REPL engine (esp_console + linenoise)
│  ├─ usb_transport.cpp/.h      // USB-CDC (OTG) transport for REPL
│  ├─ telnet_transport.cpp/.h   // TCP server transport for telnet REPL
│  └─ ws_transport.cpp/.h       // WebSocket bridge for in-browser REPL
│
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

### Benefits

* **Encapsulation**: Each feature is trivially testable and maintainable.
* **Scalability**: New features map to new folders without cluttering common code.
* **Team Autonomy**: Teams can own features end-to-end, from C++ to web assets.

### Build Integration

* Use ESP-IDF’s CMake: define each feature folder as a component.
* `main.cpp` links against `console`, `dashboard`, and `config` components.
* Static assets under `/dashboard/static/` integrate into the firmware via `esp_vfs_spiffs` or embedding.
