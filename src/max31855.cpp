#include "max31855.h"

#include "stdlib.h"

#include "Arduino.h"
#include "SPI.h"
#include "util/delay.h"

sensor::temperature::thermocouple::max31855::Driver::Driver(uint8_t channel, int8_t chip_select)
{
  _channel = channel;
  _chip_select = chip_select;

  // setup the chip select pin
  pinMode(_chip_select, OUTPUT);
  // immediately pull CS pin high to avoid conflicts on SPI bus
  digitalWrite(_chip_select, HIGH);
}

void sensor::temperature::thermocouple::max31855::Driver::update()
{
  uint32_t full_read;
  full_read = _read_from_device();	// all data is packed into 4 8-bit registers

  #ifdef DEBUG_FIRMWARE
  Serial.print("# Read SPI value: ");
  Serial.println(full_read);
  #endif

  int16_t temp_i16;
  uint8_t temp_u8;

  // un-pack chip fault status
  if(full_read & 0x00010000)
  {
    temp_u8 = (uint8_t)(full_read & 0x00000007);
    if (temp_u8 & 0x01) {
      _last_status = status::NOT_CONNECTED;
    }
    else if (temp_u8 & 0x02) {
      _last_status = status::SHORT_TO_GROUND;
    }
    else if (temp_u8 & 0x04) {
      _last_status = status::SHORT_TO_VCC;
    }
    else {
      _last_status = status::UNKNOWN;
    }
  }
  else
  {
    _last_status = status::OK;

    // un-pack TC temp data
    temp_i16 = (int16_t)(full_read >> 18);
    if(0x2000 & temp_i16)       // negative value
    {
  	  temp_i16 &= 0x1FFF;       // save only first 13 bits of data
  	  temp_i16 |= 0x8000;       // add on sign bit
    }
    else                        // positive value
    {
  	  temp_i16 &= 0x1FFF;       // no sign, save only first 13 (pos val)
    }
    // save TC temp. Note: int16 with 2 bits of
    // resolution (2^-2 = 0.25 deg C per bit)
    _last_value = temp_i16 * 0.25;
    _last_reading = micros();   
  }

  // un-pack MAX31855 internal temp data
  temp_i16 = (int16_t)(full_read >> 4);
  if(0x0800 & temp_i16)         // negative value
  {
	  temp_i16 &= 0x07FF;         // save only first 11 bits of data
	  temp_i16 |= 0x8000;         // add on sign bit
  }
  else                          // positive value
  {
	  temp_i16 &= 0x07FF;         // no sign, save only first 11 (pos val)
  }
  // save TC temp. Note: int16 with 4 bits of
  // resolution (2^-8 = 0.0625 deg C per bit)
  _last_junction_ref = temp_i16 * 0.0625;
}

inline String status_to_string(sensor::temperature::thermocouple::status status)
{
  switch(status)
  {
    case sensor::temperature::thermocouple::status::UNKNOWN: return String("unknown error");
    case sensor::temperature::thermocouple::status::OK: return String("okay");
    case sensor::temperature::thermocouple::status::NOT_CONNECTED: return String("not connected");
    case sensor::temperature::thermocouple::status::SHORT_TO_GROUND: return String("short to ground");
    case sensor::temperature::thermocouple::status::SHORT_TO_VCC: return String ("short to Vcc");
    default: return String("");
  }
}

String sensor::temperature::thermocouple::max31855::Driver::toJson()
{
  return String('{')
   + String("\"channel\": ") + String(_channel) + String(',')
   + String("\"status_code\": ") + String(static_cast<int>(_last_status), DEC)+ String(',')
   + String("\"status\": \"") + status_to_string(_last_status) + String("\",")
   + String("\"junction\": ") + String(_last_junction_ref, 3) + String(',')
   + String("\"value\": ") + String(_last_value, 3) + 
   String('}');
}

double sensor::temperature::thermocouple::max31855::Driver::getTemperature()
{
  return _last_status == status::OK ? _last_reading : -1.0;
}

double sensor::temperature::thermocouple::max31855::Driver::getJunctionReference()
{
  return _last_junction_ref;
}

sensor::temperature::thermocouple::status sensor::temperature::thermocouple::max31855::Driver::getStatus()
{
  return _last_status;
}

uint32_t sensor::temperature::thermocouple::max31855::Driver::_read_from_device(void)
{
  // Function to read 32 bits of SPI data
  uint32_t four_bytes = 0;
  
  digitalWrite(_chip_select, LOW);    // set CS low
  _delay_ms(1);                       // allow state transistion time
  
  four_bytes |= SPI.transfer(0x00);   // read 1st byte
  four_bytes <<= 8;                   // shift data 1 byte left
  four_bytes |= SPI.transfer(0x00);   // read 2nd byte
  four_bytes <<= 8;                   // shift data 1 byte left
  four_bytes |= SPI.transfer(0x00);   // read 3rd byte
  four_bytes <<= 8;                   // shift data 1 byte left
  four_bytes |= SPI.transfer(0x00);   // read 4th byte
  
  digitalWrite(_chip_select, HIGH);   // set CS high before leaving
  return four_bytes;
}