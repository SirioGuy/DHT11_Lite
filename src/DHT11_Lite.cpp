/*
*  DHT11_Lite
*  Lightweight, non-blocking DHT11 driver for AVR-based Arduino boards.
*  Uses direct port manipulation and a state machine to avoid any blocking
*  calls, keeping the MCU free between sensor phases.
*
*  Author:   Sirio Guy
*  Version:  1.0.1
*  Date:     2026
*  License:  MIT
*/


#include "DHT11_Lite.h"



// Constructor

DHT11_Lite::DHT11_Lite(uint8_t pin, uint32_t cooldownS)
    : _cooldownS(cooldownS < DHT11_COOLDOWN_S ? DHT11_COOLDOWN_S : cooldownS){


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



    for (uint8_t i = 0;  i < 5;  i++){

        _rawData[i] = 0;

    }
}



// Checksum is the low 8 bits of the sum of the first 4 bytes, per the datasheet

bool DHT11_Lite::_validateChecksum(){

    uint8_t sum = _rawData[0] + _rawData[1] + _rawData[2] + _rawData[3];

    return (sum == _rawData[4]);

}



// For DHT11 the decimal parts (rawData[1] and rawData[3]) are always 0

void DHT11_Lite::_decodeData(DHT11Data &result){

    result.humidity    = _rawData[0];

    result.temperature = (int8_t)_rawData[2];

    result.error       = false;

}




void DHT11_Lite::_resetToIdle(DHT11Data &result, bool withError){

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



// Non-blocking state machine, must be called repeatedly from loop()
// Returns true exactly once per completed reading, valid or failed

bool DHT11_Lite::read(DHT11Data &result) {


    switch (_state) {


        // Immediately begin a new measurement cycle

        case DHT11_STATE_IDLE:

          _pinModeOutput();

          _pinLow();


          _stateTimestamp = micros();

          _state = DHT11_STATE_START_LOW;

        break;



        // Hold bus low for DHT11_START_LOW_US (20 ms)

        case DHT11_STATE_START_LOW:

          if (micros() - _stateTimestamp  >=  DHT11_START_LOW_MS){

              _pinHigh();

              _pinModeInput();


              _stateTimestamp = micros();

              _state = DHT11_STATE_START_HIGH;

          }

        break;



        // MCU released the bus. We wait up to DHT11_RESPONSE_TIMEOUT_US for the DHT to pull it low as its acknowledgement

        case DHT11_STATE_START_HIGH:

          if (_pinRead() == 0){

              // Acknowledgement started

              _stateTimestamp = micros();

              _state = DHT11_STATE_RESPONSE_LOW;



          } else if (micros() - _stateTimestamp  >=  DHT11_RESPONSE_TIMEOUT_US){

              // Error: DHT11 never responded

              _resetToIdle(result, true);

              return true;

          }

        break;



        // DHT holds low for about 80us, wait for it to go high

        case DHT11_STATE_RESPONSE_LOW:

          if (_pinRead() == 1){

              // DHT released the bus

              _stateTimestamp = micros();

              _state = DHT11_STATE_RESPONSE_HIGH;



          } else if (micros() - _stateTimestamp  >=  DHT11_RESPONSE_TIMEOUT_US){

              _resetToIdle(result, true);

              return true;

          }

        break;


        // DHT holds high for about 80us before sending the first bit

        case DHT11_STATE_RESPONSE_HIGH:

          if (_pinRead() == 0){

              // First bit low phase starting

              _stateTimestamp = micros();

              _state = DHT11_STATE_BIT_LOW;



          } else if (micros() - _stateTimestamp  >=  DHT11_RESPONSE_TIMEOUT_US){

              _resetToIdle(result, true);

              return true;

          }

        break;



        // Every bit starts with a low pulse, wait for the rising edge

        case DHT11_STATE_BIT_LOW: {

          // Read the pin and the timestamp together so an interrupt firing in between can't desync them

          noInterrupts();

          uint8_t pinState = _pinRead();

          uint32_t now = micros();

          interrupts();


          if (pinState == 1){

              // High phase begins

              _bitHighStart = now;

              _state = DHT11_STATE_BIT_HIGH;



          } else if (now - _stateTimestamp  >=  DHT11_BIT_TIMEOUT_US){

              _resetToIdle(result, true);

              return true;

          }

        break; }



        // The duration of this high phase encodes the bit value: about 26 to 28us is a 0, about 70us is a 1
        // bits are received MSB first, filling rawData[0] through rawData[4]

        case DHT11_STATE_BIT_HIGH:{


          noInterrupts();

          uint8_t pinState = _pinRead();

          uint32_t now = micros();

          interrupts();


          if (pinState == 0){

              // High phase ended, measure its duration

              uint32_t highDuration = now - _bitHighStart;


              uint8_t byteIndex = _bitIndex / 8;  // which of the 5 bytes

              uint8_t bitPos    = _bitIndex % 8;  // position within that byte



              // Shift the byte left to make room, then write the new bit

              _rawData[byteIndex] <<= 1;


              if (highDuration > DHT11_BIT_THRESHOLD_US){

                  _rawData[byteIndex] |= 0x01;    // it's a '1'

              } // if <= threshold, the |= is skipped and the bit stays '0'


              _bitIndex++;


              if (_bitIndex == 40){

                  // All 40 bits received

                  if (_validateChecksum()){

                      _decodeData(result);


                  } else{

                      result.error = true;

                      result.temperature = 0;

                      result.humidity = 0;

                  }



                  // Reset counters and enter cooldown regardless of checksum result

                  _bitIndex = 0;


                  for (uint8_t i = 0;  i < 5;  i++){

                      _rawData[i] = 0;

                  }


                  _stateTimestamp = micros();
                  
                  _state = DHT11_STATE_COOLDOWN;

                  return true;    // Reading complete (valid or not — check result.error)


              } else{

                  // More bits to receive: wait for next BIT_LOW phase

                  _stateTimestamp = micros();

                  _state = DHT11_STATE_BIT_LOW;

              }


          } else if (now - _bitHighStart  >=  DHT11_BIT_TIMEOUT_US){

              _resetToIdle(result, true);

              return true;

          }

        break; }



        // DHT11 datasheet says sampling period must be >= 1 second.

        case DHT11_STATE_COOLDOWN:

          if (micros() - _stateTimestamp >= _cooldownS * 1000000UL) {

              _state = DHT11_STATE_IDLE;

          }

        break;
    }

    return false;

}