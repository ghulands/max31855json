#ifndef _GGH_MAX31855_H_
#define _GGH_MAX31855_H_

#include "stdint.h"
#include "Arduino.h"

namespace sensor {
  namespace temperature {
    namespace thermocouple {

      enum class status : int {
        UNKNOWN         = -1,
        OK              = 0,
        NOT_CONNECTED   = 1,
        SHORT_TO_GROUND = 2,
        SHORT_TO_VCC    = 3
      };
      
      namespace max31855 {
        
        using status = sensor::temperature::thermocouple::status;
        using string = String;

        class Driver {
        public:
          Driver(uint8_t channel, int8_t chip_select);
          
          void update();
          double getTemperature();
          double getJunctionReference();
          status getStatus();
          
          string toJson();
          
        private:
          uint8_t _channel;
          int8_t _chip_select;
          
          uint64_t _last_reading;
          status _last_status;
          double _last_value;
          double _last_junction_ref;
          
          uint32_t _read_from_device();
        };
        
      };
    };
  };
};

#endif // _GGH_MAX31855_H_