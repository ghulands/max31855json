
#include "max31855.h"
#include "SPI.h"
#include "EEPROM.h"
#include "Servo.h"
#include "assert.h"
#include "util/delay.h"

#define FIRMWARE_VERSION "1.0"

// Enable this if you have both boards utilizing all 8 channels.
//#define HAS_8_CHANNELS

#define DOOR_PWM_PIN 11
#define DOOR_MOVEMENT_STEPS 100.0

using tc = sensor::temperature::thermocouple::max31855::Driver;

#define CHIP_SELECT_CHANNEL_0 10
#define CHIP_SELECT_CHANNEL_1  9
#define CHIP_SELECT_CHANNEL_2  8
#define CHIP_SELECT_CHANNEL_3  7
#define CHIP_SELECT_CHANNEL_4  6
#define CHIP_SELECT_CHANNEL_5  5
#define CHIP_SELECT_CHANNEL_6  4
#define CHIP_SELECT_CHANNEL_7  3

typedef struct __sensors {
    tc tc;
    bool enabled;
} temperature_sensor;

temperature_sensor sensors[] = {
  { tc(0, CHIP_SELECT_CHANNEL_0), true },
  { tc(1, CHIP_SELECT_CHANNEL_1), true },
  { tc(2, CHIP_SELECT_CHANNEL_2), true },
  { tc(3, CHIP_SELECT_CHANNEL_3), true }

#ifdef HAS_8_CHANNELS
,
  { tc(4, CHIP_SELECT_CHANNEL_4), true },
  { tc(5, CHIP_SELECT_CHANNEL_5), true },
  { tc(6, CHIP_SELECT_CHANNEL_6), true },
  { tc(7, CHIP_SELECT_CHANNEL_7), true }
#endif
};

Servo door;

bool gStreamingTemperatureEnabled = false;
bool gStreamingStatusEnabled = false;
bool gDoorOpened = false;
uint16_t gDoorClosedPosition = 0;
uint16_t gDoorOpenPosition = 0;
uint16_t gStreamingDelay = 0;

#define MAX_CMD_BUF_SIZE 16
char gCommandBuffer[MAX_CMD_BUF_SIZE];
uint8_t gCommandBufferIndex = 0;

#ifndef HAS_8_CHANNELS
  #define NUMBER_OF_SENSORS 4
#else
  #define NUMBER_OF_SENSORS 8
#endif

#define EEPROM_SENSOR_ENABLED_OFFSET 4
#define EEPROM_STREAM_TEMPERATURE_OFFSET (EEPROM_SENSOR_ENABLED_OFFSET + NUMBER_OF_SENSORS)
#define EEPROM_STREAM_STATUS_OFFSET (EEPROM_STREAM_TEMPERATURE_OFFSET + 1)
#define EEPROM_STREAMING_DELAY_OFFSET (EEPROM_STREAM_STATUS_OFFSET + 1)
#define EEPROM_DOOR_CLOSED_OFFSET (EEPROM_STREAMING_DELAY_OFFSET + 2)
#define EEPROM_DOOR_OPEN_OFFSET (EEPROM_DOOR_CLOSED_OFFSET + 2)
#define EEPROM_DOOR_IS_OPEN_OFFSET (EEPROM_DOOR_OPEN_OFFSET + 2)

#define DEFAULT_DOOR_OPEN_POSITION 360
#define DEFAULT_STREAMING_DELAY 500


void validate_eeprom() {
    // check if eeprom is initialized.
  if (EEPROM.read(0) == 0xb
    && EEPROM.read(1) == 0xe
    && EEPROM.read(2) == 0xe
    && EEPROM.read(3) == 0xf) {
        #ifdef DEBUG_FIRMWARE
            Serial.println("# EEPROM initialized. Reading saved values.");
        #endif
        // read in stored state
        for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
            sensors[i].enabled = EEPROM.read(EEPROM_SENSOR_ENABLED_OFFSET + i) == 1;
        }
        gStreamingTemperatureEnabled = EEPROM.read(EEPROM_STREAM_TEMPERATURE_OFFSET) == 1;
        gStreamingStatusEnabled = EEPROM.read(EEPROM_STREAM_STATUS_OFFSET) == 1;

        gDoorClosedPosition = (EEPROM.read(EEPROM_DOOR_CLOSED_OFFSET + 1) << 8) | EEPROM.read(EEPROM_DOOR_CLOSED_OFFSET);
        gDoorOpenPosition = (EEPROM.read(EEPROM_DOOR_OPEN_OFFSET + 1) << 8) | EEPROM.read(EEPROM_DOOR_OPEN_OFFSET);
        gStreamingDelay = (EEPROM.read(EEPROM_STREAMING_DELAY_OFFSET + 1) << 8) | EEPROM.read(EEPROM_STREAMING_DELAY_OFFSET);

        gDoorOpened = EEPROM.read(EEPROM_DOOR_IS_OPEN_OFFSET) == 1;

        #ifdef DEBUG_FIRMWARE
            Serial.println("# Saved Values");
            for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
                Serial.print("# Sensor ");
                Serial.print(i, DEC);
                sensors[i].enabled ? Serial.println(" enabled.") : Serial.println(" disabled.");
            }
            Serial.print("# Temperature Streaming: ");
            gStreamingTemperatureEnabled ? Serial.println("enabled.") : Serial.println("disabled.");
            Serial.print("# Status Streaming: ");
            gStreamingStatusEnabled ? Serial.println("enabled.") : Serial.println("disabled.");
            Serial.print("# Streaming Delay: ");
            Serial.println(gStreamingDelay, DEC);
            Serial.print("# Door Closed Position: ");
            Serial.println(gDoorClosedPosition, DEC);
            Serial.print("# Door Open Position: ");
            Serial.println(gDoorOpenPosition, DEC);
            Serial.print("# Door State: ");
            gDoorOpened ? Serial.println("opened.") : Serial.println("closed.");
        #endif
    }
    else {
        #ifdef DEBUG_FIRMWARE
            Serial.println("# EEPROM not initialized. Setting default values.");
        #endif

        // initialize with default state
        EEPROM.write(0, 0xb);
        EEPROM.write(1, 0xe);
        EEPROM.write(2, 0xe);
        EEPROM.write(3, 0xf);

        // enable all channels
        for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
            EEPROM.write(EEPROM_SENSOR_ENABLED_OFFSET + i, 1);
        }

        EEPROM.write(EEPROM_STREAM_TEMPERATURE_OFFSET, 0); // disable temperature streaming by default
        EEPROM.write(EEPROM_STREAM_STATUS_OFFSET, 0); // disable status streaming by default
        EEPROM.write(EEPROM_DOOR_CLOSED_OFFSET, 0);
        EEPROM.write(EEPROM_DOOR_CLOSED_OFFSET + 1, 0);
        EEPROM.write(EEPROM_DOOR_OPEN_OFFSET, ((uint16_t)DEFAULT_DOOR_OPEN_POSITION));
        EEPROM.write(EEPROM_DOOR_OPEN_OFFSET + 1, ((uint16_t)DEFAULT_DOOR_OPEN_POSITION) >> 8);
        EEPROM.write(EEPROM_STREAMING_DELAY_OFFSET, ((uint16_t)DEFAULT_STREAMING_DELAY));
        EEPROM.write(EEPROM_STREAMING_DELAY_OFFSET + 1, ((uint16_t)DEFAULT_STREAMING_DELAY) >> 8);
        EEPROM.write(EEPROM_DOOR_IS_OPEN_OFFSET, 0);
    }
}

void setup()
{
  Serial.begin(115200);

  SPI.begin();
  SPI.setDataMode(SPI_MODE1);

  door.attach(DOOR_PWM_PIN);

#ifdef DEBUG_FIRMWARE
  Serial.print("# ");
  Serial.print(NUMBER_OF_SENSORS);
  Serial.print(" Channel Thermocouple Board. Version: ");
  Serial.println(FIRMWARE_VERSION);
#endif

  validate_eeprom();
}

void processCommand() {
#ifdef DEBUG_FIRMWARE
    Serial.print("# Received Command: ");
    uint8_t i = 0;
    while (i < MAX_CMD_BUF_SIZE) {
        Serial.print(gCommandBuffer[i]);
        if (gCommandBuffer[i] == ';') {
            break;
        }
        i++;
    }
    Serial.println("");
#endif

    int cmd = gCommandBuffer[0];

    switch (cmd) {
        case 'T':
        {
            // Temperature Commands
            // T E0; enable channel 0 all channels are enabled by default.
            // T D0; disable channel 0
            // T ONESHOT; read and return sensor values once.
            // T S1 500; turn streaming mode on with a delay of 500 ms between readings.
            // T S0; turn streaming mode off
            int sub_cmd = gCommandBuffer[2];

            switch (sub_cmd) {
                case 'E': {
                    int channel = gCommandBuffer[3] - '0';

                    if (channel >= NUMBER_OF_SENSORS) {
                        Serial.println("# Invalid channel number");
                        return;
                    }
                    sensors[channel].enabled = true;
                    EEPROM.write(EEPROM_SENSOR_ENABLED_OFFSET + channel, 1);

                    #ifdef DEBUG_FIRMWARE
                    Serial.print("# Enabled channel ");
                    Serial.println(channel);
                    #endif

                } return;
                case 'D': {
                    int channel = gCommandBuffer[3] - '0';

                    if (channel >= NUMBER_OF_SENSORS) {
                        Serial.println("# Invalid channel number");
                        return;
                    }
                    sensors[channel].enabled = false;
                    EEPROM.write(EEPROM_SENSOR_ENABLED_OFFSET + channel, 0);

                    #ifdef DEBUG_FIRMWARE
                    Serial.print("# Disabled channel ");
                    Serial.println(channel);
                    #endif

                } return;
                case 'O': {
                    for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
                        if(sensors[i].enabled) {
                            sensors[i].tc.update();
                        }
                    }
                    Serial.print("{ \"temperature\": [");
                    bool printed = false;
                    for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
                        if(sensors[i].enabled) {
                            if (printed) {
                                Serial.print(',');
                            }
                            Serial.print(sensors[i].tc.toJson());
                            printed = true;
                        }
                    }
                    Serial.println("]}");
                } return;
                case 'S': {
                    int state = gCommandBuffer[3] - '0';
                    
                    if (state == 0) {
                        gStreamingTemperatureEnabled = false;
                    }
                    else if (state == 1) {
                        gStreamingTemperatureEnabled = true;
                        gStreamingStatusEnabled = false;
                        long delay = strtol(&gCommandBuffer[4], NULL, 10);
                        gStreamingDelay = (uint16_t)delay;
                    }
                    else {
                        Serial.print("# Unknown state value: ");
                        Serial.print(state, DEC);
                    }
                    // Save to EEPROM
                    EEPROM.write(EEPROM_STREAM_TEMPERATURE_OFFSET, gStreamingTemperatureEnabled ? 1 : 0);
                    EEPROM.write(EEPROM_STREAMING_DELAY_OFFSET, gStreamingDelay);
                    EEPROM.write(EEPROM_STREAMING_DELAY_OFFSET + 1, gStreamingDelay >> 8);
                } return;
            }
            return;
        }
        case 'D':
        {
            // Door Controls
            // D C 23 120; configure the closed and open values for the door.
            // D O1500; open the door over a 1500 ms duration.
            // D S750; shut the door over a 750ms duration.
            uint8_t sub_command = gCommandBuffer[2];

            switch (sub_command) {
                case 'C': {
                    long temp = strtol(&gCommandBuffer[4], NULL, 10);
                    gDoorClosedPosition = (uint16_t)temp;

                    // find start of next number
                    uint8_t idx = 4;
                    while (gCommandBuffer[idx] != ' ') {
                        idx++;
                    }
                    temp = strtol(&gCommandBuffer[idx], NULL, 10);
                    gDoorOpenPosition = (uint16_t)temp;

                    EEPROM.write(EEPROM_DOOR_CLOSED_OFFSET, gDoorClosedPosition);
                    EEPROM.write(EEPROM_DOOR_CLOSED_OFFSET + 1, gDoorClosedPosition >> 8);
                    EEPROM.write(EEPROM_DOOR_OPEN_OFFSET, gDoorOpenPosition);
                    EEPROM.write(EEPROM_DOOR_OPEN_OFFSET + 1, gDoorOpenPosition >> 8);
                } return;
                case 'O': {
                    if (gDoorOpened) {
                        return; // can't open the door twice.
                    }
                    long ms = strtol(&gCommandBuffer[3], NULL, 10);
                    long pause = ms / DOOR_MOVEMENT_STEPS;
                    uint16_t step = (gDoorOpenPosition - gDoorClosedPosition) / DOOR_MOVEMENT_STEPS;
                    
                    for (uint16_t pos = gDoorClosedPosition; pos <= gDoorOpenPosition; pos += step) { 
                        door.write(pos);
                        delay(pause);
                    }
                    door.write(gDoorOpenPosition);
                    
                    gDoorOpened = true;
                    EEPROM.write(EEPROM_DOOR_IS_OPEN_OFFSET, 1);
                } return;
                case 'S': {
                    if (!gDoorOpened) {
                        return; // can't close the door twice.
                    }
                    long ms = strtol(&gCommandBuffer[3], NULL, 10);
                    long pause = ms / DOOR_MOVEMENT_STEPS;
                    uint16_t step = (gDoorOpenPosition - gDoorClosedPosition) / DOOR_MOVEMENT_STEPS;
                    
                    for (uint16_t pos = gDoorOpenPosition; pos >= gDoorClosedPosition; pos -= step) { 
                        door.write(pos);
                        delay(pause);
                    }
                    door.write(gDoorClosedPosition);
                    
                    gDoorOpened = true;
                    EEPROM.write(EEPROM_DOOR_IS_OPEN_OFFSET, 0);
                }
            }

            return;
        }
        case 'S':
        {
            // Status Control
            // S1 500; turn on streaming status. returns json of temperature and door
            // S0; turn off streaming status mode.
            uint8_t state = gCommandBuffer[3] - '0';

            if (state == 1) {
                long delay = strtol(&gCommandBuffer[4], NULL, 10);

                gStreamingDelay = (uint16_t)delay;
                gStreamingStatusEnabled = true;
                gStreamingTemperatureEnabled = false;
            }
            else if (state == 0) {
                gStreamingStatusEnabled = false;
            }
            
            // Save to EEPROM
            EEPROM.write(EEPROM_STREAM_STATUS_OFFSET, gStreamingStatusEnabled ? 1 : 0);
            EEPROM.write(EEPROM_STREAMING_DELAY_OFFSET, gStreamingDelay);
            EEPROM.write(EEPROM_STREAMING_DELAY_OFFSET + 1, gStreamingDelay >> 8);
        }
    }
}

void loop()
{
    while (Serial.available() > 0) {
        int c = Serial.read();

        gCommandBuffer[gCommandBufferIndex] = c;
        gCommandBufferIndex++;
        if (gCommandBufferIndex == MAX_CMD_BUF_SIZE) {
            gCommandBufferIndex = 0;
        }

        if (c == '\n') {
            processCommand();
            gCommandBufferIndex = 0;
        }        
    }

    if (gStreamingStatusEnabled || gStreamingTemperatureEnabled) {
        for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
            if(sensors[i].enabled) {
                sensors[i].tc.update();
            }
        }

        Serial.print("{ \"temperature\": [");
        bool printed = false;
        for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
            if(sensors[i].enabled) {
                if (printed) {
                    Serial.print(',');
                }
                Serial.print(sensors[i].tc.toJson());
                printed = true;
            }
        }
        Serial.print("]");
        if (!gStreamingStatusEnabled) {
            Serial.println("");
        }

        if (gStreamingStatusEnabled) {
            Serial.print(", \"door\": {");
            Serial.print("\"state\": \"");
            gDoorOpened ? Serial.print("open") : Serial.print("closed");
            Serial.print("\", ");
            Serial.print("\"position\": ");
            gDoorOpened ? Serial.print(gDoorOpenPosition, DEC) : Serial.print(gDoorClosedPosition);
            Serial.print("}");
        }
        Serial.println('}');

        if (gStreamingDelay > 0) {
            delay(gStreamingDelay);
        }
    }
}
