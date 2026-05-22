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

#pragma once

#ifndef ARDUINO_ARCH_AVR
    #error "This library only supports boards with an AVR processor."
#endif

#include <Arduino.h>
#include <stdint.h>


// ─── Timing constants ─────────────────────────────────────────────────────────

#define DHT11_START_LOW_MS          20      // MCU holds bus LOW to wake sensor (datasheet: >= 18ms)
#define DHT11_RESPONSE_TIMEOUT_US   200     // Max wait for any DHT response edge
#define DHT11_BIT_THRESHOLD_US      50      // HIGH pulse > 50us: bit '1', else bit '0'
#define DHT11_BIT_TIMEOUT_US        100     // Max allowed duration of a single bit pulse
#define DHT11_COOLDOWN_S            1       // Min time between readings (datasheet: >= 1s)


// ─── State machine ────────────────────────────────────────────────────────────

typedef enum : uint8_t {

    DHT11_STATE_IDLE,           // Ready to begin a new measurement
    DHT11_STATE_START_LOW,      // MCU holding bus LOW
    DHT11_STATE_START_HIGH,     // MCU released bus, polling for DHT response
    DHT11_STATE_RESPONSE_LOW,   // DHT acknowledging with ~80us LOW
    DHT11_STATE_RESPONSE_HIGH,  // DHT preparing data with ~80us HIGH
    DHT11_STATE_COOLDOWN,       // Enforcing minimum period between readings

} DHT11State;


// ─── Result struct ────────────────────────────────────────────────────────────

typedef struct {

    int8_t  temperature;    // Degrees Celsius (DHT11 range: 0 to 50)
    uint8_t humidity;       // Relative humidity % (DHT11 range: 20 to 90)
    bool    error;          // true if the reading failed for any reason

} DHT11Data;


// ─── Driver class ─────────────────────────────────────────────────────────────

class DHT11_Lite {

  public:

    DHT11_Lite(uint8_t pin, uint32_t cooldownS = DHT11_COOLDOWN_S);

    // Call repeatedly from loop(). Returns true once a reading is complete.
    // Check result.error to know whether the data is valid.
    bool read(DHT11Data &result);

  private:

    // Port registers — resolved once in the constructor via Arduino macros,
    // giving single-instruction register access with no runtime overhead.
    volatile uint8_t *_ddr;
    volatile uint8_t *_port;
    volatile uint8_t *_pin;
    uint8_t           _mask;

    // State machine
    DHT11State  _state;
    uint32_t    _stateTimestamp;
    uint32_t    _cooldownS;

    // Raw data buffer
    uint8_t  _rawData[5];       // 40 bits = 5 bytes: RH_int, RH_dec, T_int, T_dec, checksum

    // Pin control — inline keeps these as single register instructions
    inline void _pinModeOutput() { *_ddr  |=  _mask; }
    inline void _pinModeInput()  { *_ddr  &= ~_mask; }
    inline void _pinHigh()       { *_port |=  _mask; }
    inline void _pinLow()        { *_port &= ~_mask; }
    inline bool _pinRead()       { return (*_pin & _mask) != 0; }

    // Reads all 40 bits in a single blocking call (~4ms).
    // Called with interrupts disabled to protect micros() timing.
    bool _readBits();

    bool _validateChecksum();
    void _decodeData(DHT11Data &result);
    void _resetToIdle(DHT11Data &result, bool withError);

};
