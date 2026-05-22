/*
 *  DHT11_Lite
 *  Lightweight, non-blocking DHT11 driver for AVR-based Arduino boards.
 *  Uses direct port manipulation and a state machine to avoid any blocking
 *  calls, keeping the MCU free between sensor phases.
 *
 *  Author:   Sirio Guy
 *  Version:  1.1.0
 *  Date:     2026
 *  License:  MIT
 */

#include "DHT11_Lite.h"


// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// digitalPinToPort(), digitalPinToBitMask(), and the port register macros are
// resolved at compile time, so _ddr, _port, _pin and _mask are ready for
// single-instruction use throughout the driver.
// ─────────────────────────────────────────────────────────────────────────────

DHT11_Lite::DHT11_Lite(uint8_t pin, uint32_t cooldownS)
    : _cooldownS(cooldownS) {

    // Clamp to the datasheet minimum (section 6: >= 1s between readings)
    if (_cooldownS < 1) {
        _cooldownS = 1;
    }

    uint8_t port = digitalPinToPort(pin);

    _mask  = digitalPinToBitMask(pin);
    _ddr   = portModeRegister(port);
    _port  = portOutputRegister(port);
    _pin   = portInputRegister(port);

    // Start as input with no pull-up; the circuit provides its own pull-up resistor
    _pinModeInput();
    _pinLow();

    _state          = DHT11_STATE_IDLE;
    _stateTimestamp = 0;

    for (uint8_t i = 0;  i < 5;  i++) {
        _rawData[i] = 0;
    }
}


// ─────────────────────────────────────────────────────────────────────────────
// _readBits
// Reads all 40 bits from the DHT11 in a single blocking call (~4ms).
// Must be called with interrupts already disabled to protect micros() from
// Timer0 overflow corruption during bit timing.
// Returns false immediately on any timeout.
// ─────────────────────────────────────────────────────────────────────────────

bool DHT11_Lite::_readBits() {

    for (uint8_t i = 0;  i < 40;  i++) {

        uint32_t t = micros();

        // Wait for the LOW phase to end (~50us)
        while (_pinRead() == 0) {
            if (micros() - t  >=  DHT11_BIT_TIMEOUT_US)  return false;
        }

        // Measure the HIGH phase duration:
        //   ~26-28us → bit '0'
        //   ~70us    → bit '1'
        t = micros();
        while (_pinRead() == 1) {
            if (micros() - t  >=  DHT11_BIT_TIMEOUT_US)  return false;
        }

        _rawData[i / 8] <<= 1;

        if (micros() - t  >  DHT11_BIT_THRESHOLD_US) {
            _rawData[i / 8] |= 0x01;
        }
    }

    return true;
}


// ─────────────────────────────────────────────────────────────────────────────
// _validateChecksum
// The 5th byte must equal the lower 8 bits of the sum of the first four.
// ─────────────────────────────────────────────────────────────────────────────

bool DHT11_Lite::_validateChecksum() {

    uint8_t sum = _rawData[0] + _rawData[1] + _rawData[2] + _rawData[3];

    return (sum == _rawData[4]);
}


// ─────────────────────────────────────────────────────────────────────────────
// _decodeData
// Byte layout sent by DHT11:
//   [0] humidity integer   [1] humidity decimal (always 0, resolution is 1%)
//   [2] temperature integer  [3] temperature decimal (always 0, resolution is 1°C)
// ─────────────────────────────────────────────────────────────────────────────

void DHT11_Lite::_decodeData(DHT11Data &result) {

    result.humidity    = _rawData[0];
    result.temperature = (int8_t)_rawData[2];
    result.error       = false;
}


// ─────────────────────────────────────────────────────────────────────────────
// _resetToIdle
// Common exit path for all error conditions. Always enters cooldown so the
// sensor gets its mandatory rest period even after a failed read.
// ─────────────────────────────────────────────────────────────────────────────

void DHT11_Lite::_resetToIdle(DHT11Data &result, bool withError) {

    _pinModeInput();
    _pinLow();

    if (withError) {
        result.error       = true;
        result.temperature = 0;
        result.humidity    = 0;
    }

    for (uint8_t i = 0;  i < 5;  i++) {
        _rawData[i] = 0;
    }

    _stateTimestamp = millis();
    _state          = DHT11_STATE_COOLDOWN;
}


// ─────────────────────────────────────────────────────────────────────────────
// read
// Non-blocking state machine. Call from loop() on every iteration.
// Returns true exactly once per completed cycle, whether successful or not.
// Always check result.error before using the data.
//
// Timing strategy:
//   millis()  → START_LOW and COOLDOWN  (millisecond resolution is sufficient)
//   micros()  → all other states        (microsecond resolution needed for bit decoding)
// ─────────────────────────────────────────────────────────────────────────────

bool DHT11_Lite::read(DHT11Data &result) {

    switch (_state) {


        case DHT11_STATE_IDLE:

            _pinModeOutput();
            _pinLow();

            _stateTimestamp = millis();
            _state = DHT11_STATE_START_LOW;

        break;


        // Hold bus LOW for DHT11_START_LOW_MS to guarantee the sensor wakes up
        case DHT11_STATE_START_LOW:

            if (millis() - _stateTimestamp  >=  DHT11_START_LOW_MS) {

                _pinHigh();
                _pinModeInput();

                _stateTimestamp = micros();
                _state = DHT11_STATE_START_HIGH;
            }

        break;


        // Poll for the DHT pulling the bus LOW as its acknowledgement.
        // More robust than a fixed delay since the DHT can respond anywhere
        // in the 20-40us window.
        case DHT11_STATE_START_HIGH:

            if (_pinRead() == 0) {

                _stateTimestamp = micros();
                _state = DHT11_STATE_RESPONSE_LOW;

            } else if (micros() - _stateTimestamp  >=  DHT11_RESPONSE_TIMEOUT_US) {

                _resetToIdle(result, true);
                return true;
            }

        break;


        // Wait for the DHT to release the bus after its ~80us LOW acknowledgement
        case DHT11_STATE_RESPONSE_LOW:

            if (_pinRead() == 1) {

                _stateTimestamp = micros();
                _state = DHT11_STATE_RESPONSE_HIGH;

            } else if (micros() - _stateTimestamp  >=  DHT11_RESPONSE_TIMEOUT_US) {

                _resetToIdle(result, true);
                return true;
            }

        break;


        // Wait for the DHT to pull LOW again, marking the start of the first bit.
        // Once detected, read all 40 bits atomically with interrupts disabled.
        // This is the only blocking section of the driver (~4ms).
        case DHT11_STATE_RESPONSE_HIGH:

            if (_pinRead() == 0) {

                noInterrupts();
                bool success = _readBits();
                interrupts();

                if (success && _validateChecksum()) {
                    _decodeData(result);
                } else {
                    result.error       = true;
                    result.temperature = 0;
                    result.humidity    = 0;
                }

                for (uint8_t i = 0;  i < 5;  i++) {
                    _rawData[i] = 0;
                }

                _stateTimestamp = millis();
                _state = DHT11_STATE_COOLDOWN;
                return true;

            } else if (micros() - _stateTimestamp  >=  DHT11_RESPONSE_TIMEOUT_US) {

                _resetToIdle(result, true);
                return true;
            }

        break;


        case DHT11_STATE_COOLDOWN:

            if (millis() - _stateTimestamp  >=  _cooldownS * 1000) {
                _state = DHT11_STATE_IDLE;
            }

        break;
    }

    return false;
}
