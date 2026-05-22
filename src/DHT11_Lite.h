/*
*  DHT11_Lite
*  Lightweight, non-blocking DHT11 driver for AVR-based Arduino boards.
*  Uses direct port manipulation and a state machine to avoid any blocking
*  calls, keeping the MCU free between sensor phases.
*
*  Author:   Sirio Guy
*  Version:  1.0.0
*  Date:     2026
*  License:  MIT
*/



#pragma once



#ifndef ARDUINO_ARCH_AVR

    #error "This library only supports boards with an AVR processor."

#endif



#include <Arduino.h>

#include <stdint.h>



// Timing constants

#define DHT11_START_LOW_MS      20      // MCU holds bus LOW (>= 18ms)

#define DHT11_START_HIGH_US     40      // MCU releases HIGH before sampling response

#define DHT11_RESPONSE_TIMEOUT_US  200  // Max time to wait for DHT response pulse

#define DHT11_BIT_THRESHOLD_US  50      // HIGH pulse > 50us: bit 1, else bit 0

#define DHT11_BIT_TIMEOUT_US    100     // Max duration of any single bit pulse

#define DHT11_COOLDOWN_S        1    // Minimum time between readings (>= 1s)



// State machine

typedef enum : uint8_t {


    DHT11_STATE_IDLE,               // Ready to start a new measurement

    DHT11_STATE_START_LOW,          // MCU pulling bus LOW

    DHT11_STATE_START_HIGH,         // MCU released bus, waiting for DHT response

    DHT11_STATE_RESPONSE_LOW,       // DHT pulling LOW for 80us (acknowledgement)

    DHT11_STATE_RESPONSE_HIGH,      // DHT pulling HIGH for 80us (prep for data)

    DHT11_STATE_BIT_LOW,            // Start of a bit: DHT pulls LOW for 50us

    DHT11_STATE_BIT_HIGH,           // DHT pulls HIGH: duration encodes 0 or 1

    DHT11_STATE_COOLDOWN,           // Waiting before next reading is allowed


} DHT11State;



// Result struct

typedef struct {


    int8_t  temperature;    // Celsius, range 0-50 for DHT11

    uint8_t humidity;       // Relative humidity %, range 20-90 for DHT11

    bool    error;          // true if reading failed (timeout or bad checksum)


} DHT11Data;



// Main class

class DHT11_Lite {

  public:


      DHT11_Lite(uint8_t pin, uint32_t cooldownS = DHT11_COOLDOWN_S);


      // Call repeatedly from loop(). Returns true when a valid reading is ready in result.
      bool read(DHT11Data &result);


  private:

      // Port registers

      volatile uint8_t *_ddr;

      volatile uint8_t *_port;

      volatile uint8_t *_pin;

      uint8_t           _mask;



      // State machine

      DHT11State  _state;

      uint32_t    _stateTimestamp;    // Timestamp of last state transition

      uint32_t    _cooldownS;



      // Interrupts are disabled during bit reception to prevent timing errors.
      // This flag tracks whether we are currently in that critical section.
      
      bool _inCriticalSection;



      // Bit reception

      uint8_t  _rawData[5];           // 40 bits = 5 bytes: RH_int, RH_dec, T_int, T_dec, checksum

      uint8_t  _bitIndex;             // Current bit being received (0–39)

      uint32_t _bitHighStart;         // micros() when the HIGH phase of a bit started



      // Internal pin helpers

      inline void _pinModeOutput() { *_ddr  |=  _mask; }

      inline void _pinModeInput()  { *_ddr  &= ~_mask; }

      inline void _pinHigh()       { *_port |=  _mask; }

      inline void _pinLow()        { *_port &= ~_mask; }

      inline bool _pinRead()       { return (*_pin & _mask) != 0; }



      // Internal helpers

      bool _validateChecksum();

      void _decodeData(DHT11Data &result);

      void _resetToIdle(DHT11Data &result, bool withError);


};