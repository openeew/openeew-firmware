# main.cpp — Function Reference & ESP-IDF FreeRTOS Refactor Guide

> Use this as a map when porting from Arduino to ESP-IDF + FreeRTOS.

---

## Architecture Overview (Arduino today)

```
setup()
  └─ init hardware, WiFi/Eth, NTP, ADXL355, MQTT

loop()  ← single-threaded, polled
  ├─ MQTT keepalive (client.loop)
  ├─ Read ADXL355 FIFO (interrupt flag fifoFull)
  ├─ Push samples → StaLtaQue (cppQueue)
  ├─ STA/LTA earthquake detection
  ├─ SendLiveData2Cloud / Send10Seconds2Cloud (MQTT)
  ├─ NeoPixel breathing animation
  └─ Periodic NTP sync

isr_adxl()  ← IRAM ISR, sets fifoFull flag
messageReceived()  ← MQTT subscription callback
NetworkEvent()     ← WiFi/Eth event callback
```

No FreeRTOS tasks, queues, or semaphores are used in the current code.

---

## Functions

### Utility

| Function | Purpose | Notes |
|---|---|---|
| `getHeaderValue(header, headerName)` | Extract value from HTTP header string | Used in OTA HTTP parsing |

---

### Accelerometer (ADXL355)

| Function | Purpose | ESP-IDF equivalent |
|---|---|---|
| `isr_adxl()` | ISR: sets `fifoFull = true` on FIFO-full interrupt | `gpio_isr_handler_add()` + `xQueueSendFromISR()` |
| `StartADXL355()` | Init SPI, detect chip, calibrate, set ODR/range, read FIFO warm-up | Call from sensor task |

**Key globals:** `adxl355`, `fifoFull`, `fifoOut[32][3]`, `Adxl355SampleRate`, `odr_lpf`, `range`, `TrueSampleRate`

---

### Network — WiFi / Ethernet

| Function | Purpose | Notes |
|---|---|---|
| `NetworkEvent(event)` | WiFi + Ethernet event handler | Sets `bEthConnected`, `bWiFiConnected`, `bNetworkInterfaceChanged` |
| `connectToWiFi(init_str)` | Assert WiFi connected or restart | Blocking |
| `checkWiFiThenMQTT()` | Blocking: ensure WiFi then MQTT | Used at startup |
| `checkWiFiThenMQTTNonBlocking()` | Non-blocking retry with 5 s interval | Used in loop |
| `checkWiFiThenReboot()` | If disconnected and not Ethernet → reboot | Used in loop |
| `WiFiScanAndConnect()` | Scan + match stored credentials → connect | Returns bool |
| `startSmartConfig()` | ESP-Touch SmartConfig enrollment | Stores to NVM on success |

**Key globals:** `bEthConnected`, `bWiFiConnected`, `bEthConnecting`, `bNetworkInterfaceChanged`

---

### Network — NVM Credential Storage

| Function | Purpose |
|---|---|
| `numNetworksStored()` | Read count of saved SSIDs from NVS (`Preferences`) |
| `readNetworkStored(netId)` | Read SSID + password for index `netId` into `_ssid`, `_pswd` |
| `storeNetwork(ssid, pswd)` | Append new SSID/password to NVS |
| `clearNetworks()` | Erase all stored WiFi credentials |
| `numScannedNetworks()` | Run WiFi scan, print results, return count |

**Key globals:** `prefs` (Preferences), `_ssid`, `_pswd`, `networksStored`

---

### Time / NTP

| Function | Purpose | Notes |
|---|---|---|
| `NTPConnect()` | Configure SNTP, wait for valid time | Blocking wait with retry |
| `SetTimeESP32()` | Configure multiple NTP servers, re-sync | Called at startup and periodically |
| `SyncNTPtime()` | Update `NTP_timestamp_new` from system time | Called every second in loop |
| `getNTPtimestamp()` | Derive current `device_t` from NTP + `micros()` offset | Called before every MQTT publish |

**Key globals:** `NTP_timestamp`, `NTP_timestamp_new`, `time_since_NTP`, `device_t`, `periodic_timesync`

---

### MQTT

| Function | Purpose | Notes |
|---|---|---|
| `connectToMqtt(nonBlocking)` | Connect to MQTT broker with retry | `nonBlocking=true` uses timed backoff |
| `pubSubErr(MQTTErr)` | Print PubSubClient error codes | Debug helper |
| `messageReceived(topic, payload, length)` | MQTT subscription callback | Handles commands below |
| `PublishGet()` | Publish AWS IoT shadow GET request | Startup shadow fetch |
| `PublishUpdate()` | Publish device shadow state update | Called after connect |
| `SendLiveData2Cloud()` | Serialize last 32 samples → MQTT | Continuous mode |
| `Send10Seconds2Cloud()` | Serialize full 352-sample queue → MQTT | Post-event mode |

**MQTT commands handled in `messageReceived`:**

| Topic key | Action |
|---|---|
| `state.desired.alarm` | Trigger test earthquake alarm |
| `state.desired.acqmode` | Enable/disable STA/LTA mode |
| `state.desired.staltathresh` | Override shake threshold |
| `state.desired.samplerate` | Change ADXL355 sample rate (0, 31, 125 Hz) |

---

### Earthquake Detection (STA/LTA)

Implemented inline in `loop()`. Logic:

1. When `StaLtaQue` fills (352 samples = ~11 s at 31 Hz):
   - Compute STA (last 32 samples) and LTA (last 320 samples) for each axis
   - `stalta[axis] = stav[axis] / ltav[axis]`
2. If any axis ratio > `thresh` (default 4.0): set `bPossibleEarthQuake = true`
3. While earthquake active: send continuous data every second for 5 minutes (`numSecsOfAccelReadings`)

**Key globals:** `stav[3]`, `ltav[3]`, `stalta[3]`, `thresh`, `bPossibleEarthQuake`, `STALTAMODE`, `StaLtaQue`, `offset[3]`

---

### LED / Buzzer

| Function | Purpose |
|---|---|
| `NeoPixelStatus(status)` | Set all 3 NeoPixels to a solid status color |
| `NeoPixelBreathe(status)` | Animate pulsing brightness (cyan=idle, green=connected) |
| `EarthquakeAlarm(AlarmLEDColor)` | Flash LED red + sound buzzer during alarm |
| `AlarmBuzzer()` | PWM buzzer pattern (uses `ledcWrite`) |

**LED status codes:** `LED_OFF`, `LED_CONNECTED`, `LED_FIRMWARE_OTA`, `LED_CONNECT_WIFI`, `LED_CONNECT_CLOUD`, `LED_LISTEN_WIFI`, `LED_WIFI_OFF`, `LED_SAFE_MODE`, `LED_FIRMWARE_DFU`, `LED_ERROR`, `LED_ORANGE`

---

### Arduino Entry Points

| Function | Purpose |
|---|---|
| `setup()` | One-time init: serial, LED, NVS, WiFi, Ethernet, NTP, SPI, ADXL355, MQTT, buzzer PWM |
| `loop()` | Main polling loop: MQTT, FIFO read, STA/LTA, cloud publish, LED breathe, NTP sync |

---

## Data Flow

```
ADXL355 HW interrupt
  → isr_adxl() sets fifoFull = true
    → loop() reads 32 samples from FIFO
      → convert raw counts → gals (valueToGals)
        → push AccelReading{x,y,z} into StaLtaQue
          → if queue full (352):
              run STA/LTA on all 3 axes
              if ratio > thresh → earthquake detected
                → SendLiveData2Cloud()  (every 1s for 5 min)
              else (quiet)
                → SendLiveData2Cloud()  (every 1s in continuous mode)
                   Send10Seconds2Cloud() (full history on command)
          → drop oldest sample (FIFO circular)
```

---

## Memory Budget

| Buffer | Size |
|---|---|
| `StaLtaQue` (352 × AccelReading) | ~8.5 KB |
| `fifoOut[32][3]` raw FIFO | 768 B |
| JSON publish buffer (10-sec) | ~16 KB |

---

## ESP-IDF FreeRTOS Refactor Plan

### Suggested Task Decomposition

```
┌─────────────────────────────────────────────────────────┐
│  Core 0                          Core 1                  │
│                                                          │
│  sensor_task                     mqtt_task               │
│  ├─ read ADXL355 FIFO            ├─ esp_mqtt_client loop │
│  ├─ push to accel_queue          ├─ subscribe callbacks  │
│  └─ STA/LTA calculation          └─ publish waveform     │
│                                                          │
│  network_task                    led_task                │
│  ├─ WiFi/Eth management          └─ NeoPixel status/     │
│  ├─ NTP sync                        breathing animation  │
│  └─ reconnect logic                                      │
└─────────────────────────────────────────────────────────┘

Shared:
  accel_queue      (xQueueCreate)  sensor → mqtt
  event_group      (xEventGroupCreate)  network ready, earthquake flags
  config_mutex     (xSemaphoreCreateMutex)  protect thresh, samplerate
```

### ISR Change
Replace `fifoFull` flag with `xQueueSendFromISR()` directly to `accel_queue`.

---

## Recommended ESP-IDF Libraries

### Drop-in / Official Replacements

| Arduino lib | ESP-IDF replacement | Notes |
|---|---|---|
| `WiFi.h` | `esp_wifi.h` (built-in) | Use `esp_event` loop for events |
| `ETH.h` | `esp_eth.h` (built-in) | LAN8720 driver included |
| `PubSubClient` | **`esp-mqtt`** (`esp_mqtt_client.h`) | Official Espressif MQTT client, handles TLS, auto-reconnect, QoS |
| `ArduinoJson` | **`cJSON`** (built-in to IDF) or keep ArduinoJson via CMake | cJSON is zero-dependency; ArduinoJson also works with ESP-IDF |
| `Preferences` | `nvs_flash.h` + `nvs.h` (built-in) | Direct NVS API, same underlying storage |
| `configTime` / SNTP | `esp_sntp.h` (built-in) | `sntp_setoperatingmode()`, `sntp_setservername()` |
| `Adafruit_NeoPixel` | **`led_strip`** component (built-in IDF ≥ 5.0) | RMT-based, no Arduino dependency |
| `ledcWrite` buzzer | `ledc.h` (built-in) | `ledc_timer_config()`, `ledc_channel_config()` |
| `Update.h` OTA | `esp_ota_ops.h` (built-in) | `esp_https_ota()` for URL-based OTA |
| `cppQueue` | `FreeRTOS/queue.h` (`xQueueCreate`) | Native, ISR-safe, no extra lib needed |
| `esp_task_wdt.h` | `esp_task_wdt.h` (same, built-in) | No change needed |

### ADXL355 Driver
The Arduino library (`markrad/esp32-ADXL355`) uses SPI. In ESP-IDF use:
- `driver/spi_master.h` (built-in) for the SPI bus
- Port the ADXL355 driver manually or use the component: **`idf-adxl355`** (community, search ESP-IDF component registry)

### Useful Community Components (ESP-IDF Component Registry)
```
idf.py add-dependency "espressif/esp_mqtt_cxx"   # C++ MQTT wrapper
idf.py add-dependency "espressif/led_strip"       # NeoPixel via RMT
idf.py add-dependency "espressif/esp-insights"    # optional diagnostics
```

### What You Do NOT Need in ESP-IDF
- `WiFiClientSecure` — TLS is handled by `esp-tls` inside `esp-mqtt`
- `HTTPClient` — use `esp_http_client.h` (built-in)
- `soc/soc.h` brownout hack — use `esp_brownout` or configure via `menuconfig`
- `Arduino.h`, `String` class — use `std::string` or plain `char[]`
