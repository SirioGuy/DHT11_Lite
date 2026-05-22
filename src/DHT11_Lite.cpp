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


#include "DHT11_Lite.h"



// Constructor

DHT11_Lite::DHT11_Lite(uint8_t pin, uint32_t cooldownS = DHT11_COOLDOWN_S)
    : _cooldownS(cooldownS){


    uint8_t port = digitalPinToPort(pin);


    _mask  = digitalPinToBitMask(pin);

    _ddr   = portModeRegister(port);

    _port  = portOutputRegister(port);

    _pin   = portInputRegister(port);



    _pinModeInput();

    _pinLow();



    _state          = DHT11_STATE_IDLE;

    _stateTimestamp = 0;

    _bitIndex       = 0;

    _bitHighStart   = 0;

    _inCriticalSection = false;



    for (uint8_t i = 0;  i < 5;  i++){

        _rawData[i] = 0;

    }
}



// The DHT11 sends 5 bytes. The 5th is the checksum: it must equal the lower 8 bits of the sum of the first 4 bytes.

bool DHT11_Lite::_validateChecksum(){

    uint8_t sum = _rawData[0] + _rawData[1] + _rawData[2] + _rawData[3];

    return (sum == _rawData[4]);

}



// For DHT11 specifically:
//   - _rawData[0] = humidity integer part
//   - _rawData[1] = humidity decimal part (always 0 for DHT11)
//   - _rawData[2] = temperature integer part
//   - _rawData[3] = temperature decimal part (always 0 for DHT11)

void DHT11_Lite::_decodeData(DHT11Data &result){

    result.humidity    = _rawData[0];

    result.temperature = (int8_t)_rawData[2];

    result.error       = false;

}



// Centralizes the transition back to idle/cooldown.

void DHT11_Lite::_resetToIdle(DHT11Data &result, bool withError){


    // Re-enable interrupts if we bailed out during bit reception

    if (_inCriticalSection) {

        interrupts();

        _inCriticalSection = false;

    }


    
    _pinModeInput();

    _pinLow();



    if (withError){

        result.error       = true;

        result.temperature = 0;

        result.humidity    = 0;

    }



    // Reset bit reception state for next cycle

    _bitIndex = 0;

    for (uint8_t i = 0;  i < 5;  i++){

        _rawData[i] = 0;

    }



    _stateTimestamp = millis();

    _state          = DHT11_STATE_COOLDOWN;

}



// Non-blocking state machine. Must be called repeatedly from loop().
// Returns true exactly once per completed reading (valid or failed).

bool DHT11_Lite::read(DHT11Data &result) {


    switch (_state) {


        // Immediately begin a new measurement cycle.

        case DHT11_STATE_IDLE:

          _pinModeOutput();

          _pinLow();


          _stateTimestamp = millis();

          _state = DHT11_STATE_START_LOW;

        break;



        // Hold bus LOW for DHT11_START_LOW_MS (20ms).

        case DHT11_STATE_START_LOW:

          if (millis() - _stateTimestamp  >=  DHT11_START_LOW_MS){

              _pinHigh();

              _pinModeInput();


              _stateTimestamp = micros();

              _state = DHT11_STATE_START_HIGH;

          }

        break;



        // MCU released the bus. We wait up to DHT11_RESPONSE_TIMEOUT_US for the DHT to pull it LOW as its acknowledgement.

        case DHT11_STATE_START_HIGH:

          if (_pinRead() == 0){

              // DHT pulled LOW >> acknowledgement started

              _stateTimestamp = micros();

              _state = DHT11_STATE_RESPONSE_LOW;



          } else if (micros() - _stateTimestamp  >=  DHT11_RESPONSE_TIMEOUT_US){

              // DHT never responded >> error

              _resetToIdle(result, true);

              return true;

          }

        break;



        // DHT holds LOW for ~80us. We wait for it to go HIGH.

        case DHT11_STATE_RESPONSE_LOW:

          if (_pinRead() == 1){

              // DHT released the bus >> response LOW phase done

              _stateTimestamp = micros();

              _state = DHT11_STATE_RESPONSE_HIGH;



          } else if (micros() - _stateTimestamp  >=  DHT11_RESPONSE_TIMEOUT_US){

              _resetToIdle(result, true);

              return true;

          }

        break;


        // DHT holds HIGH for ~80us to prepare for data transmission.

        case DHT11_STATE_RESPONSE_HIGH:

          if (_pinRead() == 0){

              // DHT pulled LOW >> first bit LOW phase starting

              _stateTimestamp = micros();

              _state = DHT11_STATE_BIT_LOW;



          } else if (micros() - _stateTimestamp  >=  DHT11_RESPONSE_TIMEOUT_US){

              _resetToIdle(result, true);

              return true;

          }

        break;



        // Every bit starts with a ~50us LOW pulse from the DHT. We wait for the rising edge to start timing the HIGH phase.

        // ── DHT11_STATE_BIT_LOW ───────────────────────────────────────────────────
        // Disable interrupts on the first bit only, not on every bit.
        // By the time we reach bit 1..39 they are already disabled.
        case DHT11_STATE_BIT_LOW:

            if (_inCriticalSection == 0) {

                noInterrupts();

                _inCriticalSection = true;

            }


            if (_pinRead() == 1) {

                _bitHighStart = micros();

                _state = DHT11_STATE_BIT_HIGH;


            } else if (micros() - _stateTimestamp  >=  DHT11_BIT_TIMEOUT_US) {

                _resetToIdle(result, true);

                return true;

            }

        break;



        // The duration of this HIGH phase encodes the bit value:
        //   ~26-28us → bit '0'
        //   ~70us    → bit '1'
        // We use DHT11_BIT_THRESHOLD_US (50us) as the decision boundary.
        //
        // Bit packing: DHT sends MSB first.
        // _rawData[0] fills up first (bits 0-7), then [1], [2], [3], [4].
        // Within each byte, bit 0 is the MSB → we shift left and OR the new bit.

        case DHT11_STATE_BIT_HIGH:

            if (_pinRead() == 0) {

                uint32_t highDuration = micros() - _bitHighStart;

                uint8_t byteIndex = _rawData[_bitIndex / 8];    // which of the 5 bytes

                _rawData[_bitIndex / 8] <<= 1;


                if (highDuration > DHT11_BIT_THRESHOLD_US){

                    _rawData[_bitIndex / 8] |= 0x01;

                }


                _bitIndex++;


                if (_bitIndex == 40) {

                    // All bits received — safe to re-enable interrupts now

                    interrupts();

                    _inCriticalSection = false;


                    if (_validateChecksum()) {

                        _decodeData(result);


                    } else {

                        result.error       = true;

                        result.temperature = 0;

                        result.humidity    = 0;

                    }


                    _bitIndex = 0;


                    for (uint8_t i = 0;  i < 5;  i++){

                        _rawData[i] = 0;

                    }


                    _stateTimestamp = millis();

                    _state = DHT11_STATE_COOLDOWN;


                    return true;


                } else {

                    _stateTimestamp = micros();

                    _state = DHT11_STATE_BIT_LOW;

                }


            } else if (micros() - _bitHighStart  >=  DHT11_BIT_TIMEOUT_US) {

                _resetToIdle(result, true);

                return true;

            }

        break;



        // DHT11 datasheet says sampling period must be >= 1 second.

        case DHT11_STATE_COOLDOWN:

          if (millis() - _stateTimestamp >= _cooldownS * 1000) {

              _state = DHT11_STATE_IDLE;

          }

        break;
    }

    return false;

}