# DHT11_Lite

Lightweight, **non-blocking** DHT11 driver for AVR-based Arduino boards.

Instead of using `delay()` to bit-bang the DHT11's single-wire protocol, `DHT11_Lite` implements the read cycle as a state machine driven entirely by `micros()`. Call `read()` from `loop()` as often as you like — it returns immediately in almost every call, and only returns `true` once a full measurement is ready. Your `loop()` never stalls waiting on the sensor.

## Features

- **Non-blocking** — no `delay()` calls anywhere in the read path.
- **Direct port I/O** — uses AVR port registers instead of `digitalRead`/`digitalWrite` for faster, more deterministic pin access.
- **Small footprint** — no dynamic allocation, no dependencies beyond `Arduino.h`.
- **Simple API** — one constructor, one method (`read`), one result struct.

## Requirements

- An AVR-based board (Uno, Nano, Mega, Pro Mini, etc.). ATmega328P/ATmega2560 or similar.
- A DHT11 temperature/humidity sensor.

> [!WARNING]
> **AVR only.** This library accesses AVR port registers directly and will fail to compile on non-AVR boards (ESP32, ESP8266, SAMD, RP2040, etc.).
> **DHT11 only.** The DHT22/AM2302 use a different data format and are **not** compatible with this library as-is.

## Installation

### Arduino IDE
1. Download this repository as a `.zip` (Code → Download ZIP).
2. In the Arduino IDE: **Sketch → Include Library → Add .ZIP Library...**
3. Select the downloaded file.

### PlatformIO
Add to your `platformio.ini`:
```ini
lib_deps =
    https://github.com/SirioGuy/DHT11_Lite.git
```

## Usage

```cpp
#include <DHT11_Lite.h>

// DHT11 connected to digital pin 5.
// Minimum delay between measurements: 2 seconds.
DHT11_Lite sensor(5, 2);

// Stores the latest sensor reading.
DHT11Data data;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // read() must be called continuously.
  // It returns true only when a complete reading is available.
  if (sensor.read(data)) {

    if (data.error) {
      Serial.println(F("Sensor read failed"));
    } else {
      Serial.print(F("Temperature: "));
      Serial.print(data.temperature);
      Serial.print(F(" °C  Humidity: "));
      Serial.print(data.humidity);
      Serial.println(F(" %"));
    }
  }

  // Other non-blocking code can run here. loop() is never stalled.
}
```

See [`examples/Read_DHT_Example`](examples/Read_DHT_Example/Read_DHT_Example.ino) for the full sketch.

## API Reference

### `DHT11_Lite(uint8_t pin, uint32_t cooldownS = 1)`
Constructs a driver instance bound to `pin`. `cooldownS` sets the minimum time (in seconds) between the end of one reading and the start of the next. The DHT11 datasheet requires **at least 1 second** between samples.

### `bool read(DHT11Data &result)`
Advances the internal state machine by one step. Must be called repeatedly (e.g. every loop iteration). Returns `true` exactly once per completed reading cycle — check `result.error` to know whether that reading succeeded. Returns `false` on every other call, including while a measurement is still in progress.

### `struct DHT11Data`
| Field | Type | Description |
|---|---|---|
| `temperature` | `int8_t` | Temperature in °C (0–50 range for DHT11) |
| `humidity` | `uint8_t` | Relative humidity in % (20–90 range for DHT11) |
| `error` | `bool` | `true` if the reading failed (timeout or checksum mismatch) |

## How it works

The DHT11 protocol is a single-wire, timing-based exchange:

1. The MCU pulls the line LOW for ≥18ms to signal a request, then releases it.
2. The DHT11 acknowledges with an 80µs LOW pulse followed by an 80µs HIGH pulse.
3. The DHT11 then sends 40 bits (5 bytes: humidity int, humidity dec, temp int, temp dec, checksum). Each bit starts with a ~50µs LOW pulse, followed by a HIGH pulse whose **duration** encodes the value: ~26–28µs for a `0`, ~70µs for a `1`.

`DHT11_Lite` models each of these phases as a state (`DHT11_STATE_START_LOW`, `DHT11_STATE_RESPONSE_HIGH`, `DHT11_STATE_BIT_HIGH`, etc.) and advances between them based on pin transitions and elapsed time, so no phase ever blocks the CPU while waiting.

## Known limitations

- **DHT11 only** — no support for DHT22/AM2302 (different data resolution and decoding).
- **AVR only** — relies on direct port register access (`portModeRegister`, etc.).
- Bit-timing windows are tight relative to `micros()`'s ~4µs resolution on 16MHz AVR parts; an interrupt firing mid-pulse can occasionally distort a reading.

## License

MIT — see [LICENSE](LICENSE).

## Author

Sirio Guy — [lu3alt@gmail.com](mailto:lu3alt@gmail.com)