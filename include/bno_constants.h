#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <cstdint>

/*
 These are constants for the communication with the sensor HUB
 over the SHTP-Protocol
*/

struct sh2_command_t {

    uint8_t id;
    uint8_t report_length;
};

struct bno_sensor_t {

    uint8_t id;
    uint8_t report_length;
    float   scale_factor;
};

namespace bno_constants
{   

    namespace driver_config
    {
        static constexpr uint32_t     READ_TASK_STACK_SIZE = 4096;
        static constexpr UBaseType_t  READ_TASK_PRIORITY   =    3;
        static constexpr uint8_t      BNO_SENSOR_UNDEFINED =    0;
        static constexpr uint8_t      FRS_READ_BUFFER_SIZE =   64;  // Words
        static constexpr TickType_t   BNO_TICKS_TO_TIMEOUT =  pdMS_TO_TICKS(100);
    }

    namespace i2c 
    {   
        static constexpr uint8_t  DEFAULT_ADDRESS     = 0x4B;
        static constexpr uint8_t  ALTERNATE_ADDRESS   = 0x4A; 
        static constexpr uint32_t DEFAULT_CLOCK_SPEED = 100'000;

        static constexpr int MAX_I2C_RX_BUFFER        = 128;
        static constexpr int MAX_I2C_TX_BUFFER        = 128;
        static constexpr int I2C_TIMEOUT_MS           = 100;
        static constexpr int CLOCK_STRETCH_WAIT_US    = 5000;
    }

    namespace shtp
    {
        static constexpr int SHTP_HEADER_SIZE            = 4;   // Bytes
        static constexpr int SHTP_PACKET_BUFFER_SIZE     = 512; // Bytes
    }

    namespace executable_command
    {
        static constexpr uint8_t RESERVED  = 0;
        static constexpr uint8_t RESET     = 1;
        static constexpr uint8_t ON        = 2;
        static constexpr uint8_t SLEEP     = 3;
        static constexpr uint8_t MAX_VALUE = 3;
    }

    namespace channel
    {
        static constexpr uint8_t COMMAND                  = 0x00;
        static constexpr uint8_t EXECUTABLE               = 0x01; //Report Length of 5
        static constexpr uint8_t CONTROL                  = 0x02;
        static constexpr uint8_t REPORTS                  = 0x03;
        static constexpr uint8_t WAKE_REPORTS             = 0x04;
        static constexpr uint8_t GYRO                     = 0x05;
    }

    namespace control_type 
    {
        namespace id
        {
            static constexpr uint8_t FORCE_SENSOR_FLUSH       = 0xF0;
            static constexpr uint8_t COMMAND_RESPONSE         = 0xF1;
            static constexpr uint8_t COMMAND_REQUEST          = 0xF2;
            static constexpr uint8_t FRS_READ_RESPONSE        = 0xF3;
            static constexpr uint8_t FRS_READ_REQUEST         = 0xF4;
            static constexpr uint8_t FRS_WRITE_RESPONSE       = 0xF5;
            static constexpr uint8_t FRS_WRITE_DATA_REQUEST   = 0xF6;
            static constexpr uint8_t FRS_WRITE_REQUEST        = 0xF7;
            static constexpr uint8_t PRODUCT_ID_RESPONSE      = 0xF8;
            static constexpr uint8_t PRODUCT_ID_REQUEST       = 0xF9;
            static constexpr uint8_t SET_FEATURE_COMMAND      = 0xFD;
            static constexpr uint8_t GET_FEATURE_REQUEST      = 0xFE;
            static constexpr uint8_t GET_FEATURE_RESPONSE     = 0xFC;
        }

        namespace packet_size
        {
            static constexpr uint8_t SET_FEATURE_COMMAND    = 21;
            static constexpr uint8_t GET_FEATURE_REQUEST    =  6;
            static constexpr uint8_t GET_FEATURE_RESPONSE   = 21;
            static constexpr uint8_t COMMAND_REQUEST        = 16;
            static constexpr uint8_t FRS_WRITE_REQUEST      = 10;
            static constexpr uint8_t FRS_WRITE_DATA_REQUEST = 16;
            static constexpr uint8_t FRS_WRITE_RESPONSE     =  8;
            static constexpr uint8_t FRS_READ_REQUEST       = 12;
            static constexpr uint8_t FRS_READ_RESPONSE      = 20;
            static constexpr uint8_t EXECUTABLE             =  5;
            static constexpr uint8_t PRODUCT_ID_REQUEST     =  6;
            static constexpr uint8_t PRODUCT_ID_RESPONSE    = 20;
        }  
    }

    namespace command
    {
        static constexpr sh2_command_t ERRORS                      = {.id=0x01, .report_length=0};
        static constexpr sh2_command_t COUNTER                     = {.id=0x02, .report_length=0};
        static constexpr sh2_command_t TARE                        = {.id=0x03, .report_length=0}; // Does not return anything
        static constexpr sh2_command_t INITIALIZE                  = {.id=0x04, .report_length=20};
        static constexpr sh2_command_t DCD                         = {.id=0x06, .report_length=0};
        static constexpr sh2_command_t ME_CALIBRATION              = {.id=0x07, .report_length=20};
        static constexpr sh2_command_t DCD_PERIODIC_SAVE           = {.id=0x09, .report_length=0};
        static constexpr sh2_command_t OSCILLATOR                  = {.id=0x0A, .report_length=20};
        static constexpr sh2_command_t CLEAR_DCD_RESET             = {.id=0x0B, .report_length=0};
        static constexpr sh2_command_t TURNTABLE_CALIBRATION       = {.id=0x0C, .report_length=0}; // Not supported
        static constexpr sh2_command_t BOOTLOADER                  = {.id=0x0D, .report_length=0};
        static constexpr sh2_command_t INTERACTIVE_CALIBRATION     = {.id=0x0E, .report_length=0};

        static constexpr size_t COMMAND_PARAMETER_AMOUNT        = 9;

        namespace subcommand
        {   
            namespace tare
            {
                static constexpr uint8_t TARE_NOW          = 0x00;
                static constexpr uint8_t PERSIST_TARE      = 0x01;
                static constexpr uint8_t SET_REORIENTATION = 0x02;
            }

            namespace me_calibration
            {
                static constexpr uint8_t CONFIGURE_ME_CALIBRATION = 0x00;
                static constexpr uint8_t GET_ME_CALIBRATION       = 0x01;
            }

        }
    }

    namespace frs
    {
        namespace config
        {   
            static constexpr uint16_t STATIC_CALIBRATION_AGM                              = 0x7979;
            static constexpr uint16_t NOMINAL_CALIBRATION_AGM                             = 0x4D4D;
            static constexpr uint16_t STATIC_CALIBRATION_SRA                              = 0x8A8A;
            static constexpr uint16_t NOMINAL_CALIBRATION_SRA                             = 0x4E4E;
            static constexpr uint16_t DYNAMIC_CALIBRATION                                 = 0x1F1F;
            static constexpr uint16_t MOTION_ENGINE_POWER_MANAGEMENT_STABILITY_CLASSIFIER = 0xD3E2;
            static constexpr uint16_t SYSTEM_ORIENTATION                                  = 0x2D3E;
            static constexpr uint16_t PRIMARY_ACCEL_ORIENTATION                           = 0x2D41;
            static constexpr uint16_t SCREEN_ROTATION_ACCEL_ORIENTATION                   = 0x2D43;
            static constexpr uint16_t GYRO_ORIENTATION                                    = 0x2D46;
            static constexpr uint16_t MAG_ORIENTATION                                     = 0x2D4C;
            static constexpr uint16_t AR_VR_STABILIZATION_ROTATION_VECTOR                 = 0x3E2D;
            static constexpr uint16_t AR_VR_STABILIZATION_GAME_ROTATION_VECTOR            = 0x3E2E;
            static constexpr uint16_t SIGNIFICANT_MOTION_DETECTOR                         = 0xC274;
            static constexpr uint16_t SHAKE_DETECTOR                                      = 0x7D7D;
            static constexpr uint16_t MAX_FUSION_PERIOD                                   = 0xD7D7;
            static constexpr uint16_t SERIAL_NUMBER                                       = 0x4B4B;
            static constexpr uint16_t ENV_SENSOR_PRESSURE_CALIBRATION                     = 0x39AF;
            static constexpr uint16_t ENV_SENSOR_TEMPERATURE_CALIBRATION                  = 0x4D20;
            static constexpr uint16_t ENV_SENSOR_HUMIDITY_CALIBRATION                     = 0x1AC9;
            static constexpr uint16_t PICKUP_DETECTOR                                     = 0x1B2A;
            static constexpr uint16_t FLIP_DETECTOR                                       = 0xFC94;
            static constexpr uint16_t STABILITY_DETECTOR                                  = 0xED85;
            static constexpr uint16_t ACTIVITY_TRACKER                                    = 0xED88;
            static constexpr uint16_t SLEEP_DETECTOR                                      = 0xED87;
            static constexpr uint16_t TILT_DETECTOR                                       = 0xED89;
            static constexpr uint16_t POCKET_DETECTOR                                     = 0xEF27;
            static constexpr uint16_t CIRCLE_DETECTOR                                     = 0xEE51;
            static constexpr uint16_t USER_RECORD                                         = 0x74B4;
            static constexpr uint16_t MOTION_ENGINE_TIME_SOURCE                           = 0xD403;
            static constexpr uint16_t UART_OUTPUT_FORMAT                                  = 0xA1A1;
            static constexpr uint16_t GYRO_INTEGRATED_ROT_VEC                             = 0xA1A2;
            static constexpr uint16_t FUSION_CONTROL_FLAGS                                = 0xA1A3;
            static constexpr uint16_t SIMPLE_CALIBRATION                                  = 0xA1A4;
            static constexpr uint16_t NOMINAL_SIMPLE_CALIBRATION                          = 0XA1A5;
        }

        namespace write_status
        {
            static constexpr uint8_t WORDS_RECEIVED                        = 0x00;
            static constexpr uint8_t UNRECOGNIZED_FRS_TYPE                 = 0x01;
            static constexpr uint8_t BUSY                                  = 0x02;
            static constexpr uint8_t WRITE_COMPLETED                       = 0x03;
            static constexpr uint8_t WRITE_MODE_ENTERED                    = 0x04;
            static constexpr uint8_t WRITE_FAILED                          = 0x05;
            static constexpr uint8_t DATA_RECEIVED_WHILE_NOT_IN_WRITE_MODE = 0x06;
            static constexpr uint8_t INVALID_LENGTH                        = 0x07;
            static constexpr uint8_t RECORD_VALID                          = 0x08;
            static constexpr uint8_t RECORD_INVALID                        = 0x09;
            static constexpr uint8_t DEVICE_ERROR                          = 0x0A;
            static constexpr uint8_t RECORD_IS_READ_ONLY                   = 0x0B;
            static constexpr uint8_t UNABLE_TO_WRITE_RECORD                = 0x0C;
            
            // Internal status: RECORD_VALID followed by WRITE_COMPLETED
            static constexpr uint8_t RECORD_VALID_AND_WRITE_COMPLETED      = 0x0F; 
        }

        namespace read_status
        {
            static constexpr uint8_t NO_ERROR              = 0x00;
            static constexpr uint8_t UNRECOGNIZED_FRS_TYPE = 0x01;
            static constexpr uint8_t BUSY                  = 0x02;
            static constexpr uint8_t READ_RECORD_COMPLETED = 0x03;
            static constexpr uint8_t RECORD_EMPTY          = 0x05;
            static constexpr uint8_t DEVICE_ERROR          = 0x08;
        }
    }

    namespace advertisement_packet
    {
        static constexpr uint8_t TAG_GUID                             = 0x01;
        static constexpr uint8_t TAG_GUID_LENGTH                      = 0x04;
        static constexpr uint8_t TAG_VERSION                          = 0x80;
        static constexpr uint8_t TAG_APP_NAME                         = 0x08;
        static constexpr uint8_t TAG_MAX_CARGO_PLUS_HEADER_WRITE      = 0x02;
        static constexpr uint8_t TAG_MAX_CARGO_PLUS_HEADER_READ       = 0x03;
        static constexpr uint8_t TAG_MAX_CARGO_PLUS_HEADER_WRITE_SIZE = 0x02;
        static constexpr uint8_t TAG_MAX_CARGO_PLUS_HEADER_READ_SIZE  = 0x02;

        static constexpr uint8_t GUID_SHTP                            = 0x00;
        static constexpr uint8_t GUID_SENSORHUB                       = 0x02; 

        static constexpr uint8_t CHAR_INPUT_BUFFER_SIZE               =   16;
    }

    // This bitmap serves for internal communication, outside of the sensor (bitmasks)
    namespace bitmask_config 
    {
        static constexpr uint64_t RESET_RESPONSE_EXECUTABLE              = (uint64_t)1 << 0;
        static constexpr uint64_t ADVERTISEMENT_PACKET                   = (uint64_t)1 << 1;
        static constexpr uint64_t COMMAND_INITIALIZED                    = (uint64_t)1 << 2;
        static constexpr uint64_t GET_FEATURE_RESPONSE                   = (uint64_t)1 << 3;
        static constexpr uint64_t FRS_READ_COMPLETE                      = (uint64_t)1 << 4;
        static constexpr uint64_t FRS_WRITE_RESPONSE                     = (uint64_t)1 << 5;
        static constexpr uint64_t PRODUCT_ID_RESPONSE                    = (uint64_t)1 << 6;
        static constexpr uint64_t COMMAND_ME_CALIBRATION_RESPONSE        = (uint64_t)1 << 7;
        static constexpr uint64_t COMMAND_OSCILLATOR_TYPE_RESPONSE       = (uint64_t)1 << 8;
    } 

    // This bitmap is sent to the sensor (bit positions)
    namespace feature_flags 
    {   
        static constexpr uint8_t CHANGE_SENSITIVITY_RELATIVE = (uint8_t)1 << 0;
        static constexpr uint8_t CHANGE_SENSITIVITY_ENABLED  = (uint8_t)1 << 1;
        static constexpr uint8_t WAKEUP_ENABLED              = (uint8_t)1 << 2;
        static constexpr uint8_t ALWAYS_ON_ENABLED           = (uint8_t)1 << 3;
    }

    namespace scale_factor 
    {
        static constexpr float MAGNETOMETER                                = 1.0f / (1 <<  4); // Q-Point =  4
        static constexpr float PROXIMITY                                   = 1.0f / (1 <<  4); // Q-Point =  4
        static constexpr float TEMPERATURE                                 = 1.0f / (1 <<  7); // Q-Point =  7
        static constexpr float ACCELEROMETER                               = 1.0f / (1 <<  8); // Q-Point =  8
        static constexpr float AMBIENT_LIGHT                               = 1.0f / (1 <<  8); // Q-Point =  8
        static constexpr float HUMIDITY                                    = 1.0f / (1 <<  8); // Q-Point =  8
        static constexpr float GYROSCOPE                                   = 1.0f / (1 <<  9); // Q-Point =  9
        static constexpr float ANGULAR_VELOCITY                            = 1.0f / (1 << 10); // Q-Point = 10
        static constexpr float ROTATION_VECTOR_ACCURACY                    = 1.0f / (1 << 12); // Q-Point = 12
        static constexpr float ROTATION_VECTOR                             = 1.0f / (1 << 14); // Q-Point = 14
        static constexpr float ROTATION_VECTOR_CHANGE_SENSITIVITY          = 1.0f / (1 << 13); // Q-Point = 13
        static constexpr float PRESSURE                                    = 1.0f / (1 << 20); // Q-Point = 20
        static constexpr float PRESSURE_CHANGE_SENSITIVITY                 = 1.0f / (1 <<  6); // Q-Point =  6
        static constexpr float ENV_SENSOR_SCALE_FACTOR                     = 1.0f / (1 << 15); // Q-Point = 15
        static constexpr float STABILITY_CLASSIFIER_DELTA_ORIENTATION      = 1.0f / (1 << 28); // Q-Point = 28
        static constexpr float STABILITY_CLASSIFIER_STABLE_THRESHOLD       = 1.0f / (1 << 25); // Q-Point = 25
        static constexpr float SIGNIFICANT_MOTION_DETECTOR_ACCEL_THRESHOLD = 1.0f / (1 << 24); // Q-Point = 24
    }

    namespace sensor
    {
        static constexpr bno_sensor_t ACCELEROMETER                         = {.id=0x01, .report_length=19, .scale_factor=scale_factor::ACCELEROMETER  };
        static constexpr bno_sensor_t GYROSCOPE                             = {.id=0x02, .report_length=19, .scale_factor=scale_factor::GYROSCOPE      };
        static constexpr bno_sensor_t MAGNETIC_FIELD                        = {.id=0x03, .report_length=19, .scale_factor=scale_factor::MAGNETOMETER   };
        static constexpr bno_sensor_t LINEAR_ACCELERATION                   = {.id=0x04, .report_length=19, .scale_factor=scale_factor::ACCELEROMETER  }; //Acceleration without Gravity            | returns Accelerations | max. 400Hz 
        static constexpr bno_sensor_t ROTATION_VECTOR                       = {.id=0x05, .report_length=23, .scale_factor=scale_factor::ROTATION_VECTOR}; //Quaternion from Accel, Gyro and Mag     | returns Quaternions   | max. 400Hz 
        static constexpr bno_sensor_t GRAVITY                               = {.id=0x06, .report_length=19, .scale_factor=scale_factor::ACCELEROMETER  }; //Acceleration that only includes Gravity | returns Accelerations | max. 400Hz 
        static constexpr bno_sensor_t UNCALIBRATED_GYRO                     = {.id=0x07, .report_length=25, .scale_factor=scale_factor::GYROSCOPE      };
        static constexpr bno_sensor_t GAME_ROTATION_VECTOR                  = {.id=0x08, .report_length=21, .scale_factor=scale_factor::ROTATION_VECTOR}; //Quaternion from Accel and Gyro          | returns Quaternions   | max. 400Hz  
        static constexpr bno_sensor_t GEOMAGNETIC_ROTATION_VECTOR           = {.id=0x09, .report_length=23, .scale_factor=scale_factor::ROTATION_VECTOR}; //Quaternion from Accel and Mag           | returns Quaternions   | max.  90Hz  
        static constexpr bno_sensor_t PRESSURE                              = {.id=0x0A, .report_length=17, .scale_factor=scale_factor::PRESSURE       };
        static constexpr bno_sensor_t AMBIENT_LIGHT                         = {.id=0x0B, .report_length=17, .scale_factor=scale_factor::AMBIENT_LIGHT  }; //Dont have the Env Sensor 
        static constexpr bno_sensor_t HUMIDITY                              = {.id=0x0C, .report_length=15, .scale_factor=scale_factor::HUMIDITY       };
        static constexpr bno_sensor_t PROXIMITY                             = {.id=0x0D, .report_length= 0, .scale_factor=scale_factor::PROXIMITY      }; //Dont have the Env Sensor 
        static constexpr bno_sensor_t TEMPERATURE                           = {.id=0x0E, .report_length=15, .scale_factor=scale_factor::TEMPERATURE    };
        static constexpr bno_sensor_t UNCALIBRATED_MAGNETIC_FIELD           = {.id=0x0F, .report_length=25, .scale_factor=scale_factor::MAGNETOMETER   };
        static constexpr bno_sensor_t TAP_DETECTOR                          = {.id=0x10, .report_length=14, .scale_factor= -1.0f};
        static constexpr bno_sensor_t STEP_COUNTER                          = {.id=0x11, .report_length=21, .scale_factor=  1.0f};
        static constexpr bno_sensor_t SIGNIFICANT_MOTION_DETECTOR           = {.id=0x12, .report_length=15, .scale_factor= -1.0f};
        static constexpr bno_sensor_t STABILITY_CLASSIFIER                  = {.id=0x13, .report_length=15, .scale_factor= -1.0f};
        static constexpr bno_sensor_t RAW_ACCELEROMETER                     = {.id=0x14, .report_length=25, .scale_factor= -1.0f};
        static constexpr bno_sensor_t RAW_GYROSCOPE                         = {.id=0x15, .report_length=25, .scale_factor= -1.0f};
        static constexpr bno_sensor_t RAW_MAGNETOMETER                      = {.id=0x16, .report_length=25, .scale_factor= -1.0f};
        static constexpr bno_sensor_t STEP_DETECTOR                         = {.id=0x18, .report_length=17, .scale_factor= -1.0f};
        static constexpr bno_sensor_t SHAKE_DETECTOR                        = {.id=0x19, .report_length= 0, .scale_factor= -1.0f};
        static constexpr bno_sensor_t FLIP_DETECTOR                         = {.id=0x1A, .report_length= 0, .scale_factor= -1.0f};
        static constexpr bno_sensor_t PICKUP_DETECTOR                       = {.id=0x1B, .report_length= 0, .scale_factor= -1.0f};
        static constexpr bno_sensor_t STABILITY_DETECTOR                    = {.id=0x1C, .report_length=15, .scale_factor= -1.0f};
        static constexpr bno_sensor_t PERSONAL_ACTIVITY_CLASSIFIER          = {.id=0x1E, .report_length= 0, .scale_factor= -1.0f};
        static constexpr bno_sensor_t SLEEP_DETECTOR                        = {.id=0x1F, .report_length= 0, .scale_factor= -1.0f};
        static constexpr bno_sensor_t TILT_DETECTOR                         = {.id=0x20, .report_length= 0, .scale_factor= -1.0f};
        static constexpr bno_sensor_t POCKET_DETECTOR                       = {.id=0x21, .report_length= 0, .scale_factor= -1.0f};
        static constexpr bno_sensor_t CIRCLE_DETECTOR                       = {.id=0x22, .report_length= 0, .scale_factor= -1.0f};
        static constexpr bno_sensor_t HEART_RATE_MONITOR                    = {.id=0x23, .report_length= 0, .scale_factor= -1.0f};
        static constexpr bno_sensor_t AR_VR_STABILIZED_ROTATION_VECTOR      = {.id=0x28, .report_length=23, .scale_factor=scale_factor::ROTATION_VECTOR}; //Quaternion from Accel, Gyro and Mag      | returns Quaternions  | max. 400Hz 
        static constexpr bno_sensor_t AR_VR_STABILIZED_GAME_ROTATION_VECTOR = {.id=0x29, .report_length=21, .scale_factor=scale_factor::ROTATION_VECTOR}; //Quaternion from Accel and Gyro           | returns Quaternions  | max. 400Hz 
        static constexpr bno_sensor_t GYRO_INTEGRATED_ROTATION_VECTOR       = {.id=0x2A, .report_length=18, .scale_factor=scale_factor::ROTATION_VECTOR}; //Quaternion from Gyro                     | returns Quaternions  | max.  1kHz | also output angular velocity Scale Factor of 12
    }

    namespace data
    {   
        namespace sensor
        {   
            namespace accuracy
            {
                static constexpr uint8_t UNRELIABLE = 0x00;
                static constexpr uint8_t LOW        = 0x01;
                static constexpr uint8_t MEDIUM     = 0x02;
                static constexpr uint8_t HIGH       = 0x03;
                static constexpr uint8_t UNKNOWN    = 0x04;
            }

            namespace stability_classifier 
            {
                static constexpr uint8_t UNKNOWN    = 0x00;
                static constexpr uint8_t ON_TABLE   = 0x01;
                static constexpr uint8_t STATIONARY = 0x02;
                static constexpr uint8_t STABLE     = 0x03;
                static constexpr uint8_t IN_MOTION  = 0x04;
                static constexpr uint8_t MAX_VALUE  = 0x04;
            }

        }

        namespace command
        {
            namespace initialization_state
            {
                static constexpr uint8_t SUCCESSFUL       = 0x00;
                static constexpr uint8_t OPERATION_FAILED = 0x01;

                static constexpr uint8_t UNKNOWN          = 0xFF;
            }

            namespace oscillator_type
            {
                static constexpr uint8_t INTERNAL_OSCILLATOR = 0x00;
                static constexpr uint8_t EXTERNAL_CRYSTAL    = 0x01;
                static constexpr uint8_t EXTERNAL_CLOCK      = 0x02;

                static constexpr uint8_t UNKNOWN          = 0xFF;
            }

            namespace tare
            {
                namespace axis
                {
                    static constexpr uint8_t X_Y_Z = 0x07;
                    static constexpr uint8_t Z     = 0x04;
                }

                namespace sensor
                {
                    static constexpr uint8_t ROTATION_VECTOR                       = 0x00;
                    static constexpr uint8_t GAME_ROTATION_VECTOR                  = 0x01;
                    static constexpr uint8_t GEOMAGNETIC_ROTATION_VECTOR           = 0x02;
                    static constexpr uint8_t GYRO_INTEGRATED_ROTATION_VECTOR       = 0x03;
                    static constexpr uint8_t AR_VR_STABILIZED_ROTATION_VECTOR      = 0x04;
                    static constexpr uint8_t AR_VR_STABILIZED_GAME_ROTATION_VECTOR = 0x05;
                }
            }
        }
    }

    namespace batching 
    {
        static constexpr uint8_t BASE_TIMESTAMP_REFERENCE_ID                     = 0xFB;
        static constexpr uint8_t BASE_TIMESTAMP_REFERENCE_MICROSECONDS_PER_VALUE = 100;
        static constexpr uint8_t TIMESTAMP_REBASE                                = 0xFA;
        static constexpr uint8_t TIMESTAMP_REBASE_MICROSECONDS_PER_VALUE         = 100;
    }

    namespace data_offset
    {
        namespace report
        {   
            namespace metadata 
            {
                static constexpr uint8_t BASE_TIMESTAMP_REFERENCE     =  1;
                static constexpr uint8_t REPORT_ID                    =  5;
                static constexpr uint8_t SEQUENCE_NUMBER              =  6;
                static constexpr uint8_t ACCURACY_STATUS              =  7;
                static constexpr uint8_t REPORT_DELAY                 =  8;
            }
            
            namespace generic
            {
                static constexpr uint8_t SENSOR_DATA_START = 9;
            }

            namespace quaternion
            {
                static constexpr uint8_t X        =  9;
                static constexpr uint8_t Y        = 11;
                static constexpr uint8_t Z        = 13;
                static constexpr uint8_t W        = 15;
                static constexpr uint8_t ACCURACY = 17;
            }

            namespace accel_gyro_mag
            {
                static constexpr uint8_t X        =  9;
                static constexpr uint8_t Y        = 11;
                static constexpr uint8_t Z        = 13;
                
                static constexpr uint8_t BIAS_X   = 15;
                static constexpr uint8_t BIAS_Y   = 17;
                static constexpr uint8_t BIAS_Z   = 19;

                static constexpr uint8_t RAW_GYRO_TEMP = 15;
                static constexpr uint8_t RAW_IMESTAMP  = 17;
            }

            namespace step_counter
            {
                static constexpr uint8_t DETECT_LATENCY =  9;
                static constexpr uint8_t STEP_AMOUNT    = 13;
            }
            
            namespace gyro_integrated_rotation_vector
            {   
                namespace quaternion
                {
                    static constexpr uint8_t X = 0;
                    static constexpr uint8_t Y = 2;
                    static constexpr uint8_t Z = 4;
                    static constexpr uint8_t W = 6;
                }
                namespace angular_velocity
                {
                    static constexpr uint8_t X =  8;
                    static constexpr uint8_t Y = 10;
                    static constexpr uint8_t Z = 12;
                }
            }
        }

        namespace config 
        {   
            namespace frs_write
            {
                static constexpr uint8_t STATUS_CODE =  1;
                static constexpr uint8_t WORD_OFFSET =  2;
            }

            namespace frs_read
            {
                static constexpr uint8_t DATA_LENGTH_AND_STATUS  =  1;
                static constexpr uint8_t WORD_OFFSET             =  2;
                static constexpr uint8_t DATA_0                  =  4;
                static constexpr uint8_t DATA_1                  =  8;
                static constexpr uint8_t FRS_TYPE                = 12;
            }

            namespace feature_response
            {
                static constexpr uint8_t REPORT_ID                     = 1;
                static constexpr uint8_t FEATURE_FLAGS                 = 2;
                static constexpr uint8_t CHANGE_SENSITIVITY            = 3;
                static constexpr uint8_t REPORT_INTERVAL               = 5;
                static constexpr uint8_t BATCH_INTERVAL                = 9;
                static constexpr uint8_t SENSOR_SPESIFIC_CONFIGURATION = 13;
            }

            namespace product_id_response
            {
                static constexpr uint8_t RESET_CAUSE      =  1;
                static constexpr uint8_t SW_VERSION_MAJOR =  2;
                static constexpr uint8_t SW_VERSION_MINOR =  3;
                static constexpr uint8_t SW_PART_NUMBER   =  4;
                static constexpr uint8_t SW_BUILD_NUMBER  =  8;
                static constexpr uint8_t SW_VERSION_PATCH = 12;
            }

            namespace command 
            {
                // P_ == Position in the parameter array that is sent to the sensor
                // R_ == Position in the response packet (not only in the parameters array)
                // P_ is sent, R_ is received

                namespace metadata
                {
                    static constexpr uint8_t R_SEQUENCE_NUMBER          = 1;
                    static constexpr uint8_t R_COMMAND_ID               = 2;
                    static constexpr uint8_t R_COMMAND_SEQUENCE_NUMBER  = 3;
                    static constexpr uint8_t R_RESPONSE_SEQUENCE_NUMBER = 4;
                }

                namespace tare
                {
                    static constexpr uint8_t P_SUBCOMMAND    = 0;
                    static constexpr uint8_t P_BITMAP_AXIS   = 1;
                    static constexpr uint8_t P_SENSOR        = 2;
                    static constexpr uint8_t P_REORIENTATION_QUATERNION_START = 1;
                }

                namespace initialized
                {
                    static constexpr uint8_t R_STATUS = 5;
                }

                namespace oscillator
                {
                    static constexpr uint8_t R_TYPE = 5;
                }

                namespace me_calibration
                {
                    static constexpr uint8_t P_ACCEL_CAL_ENABLE        = 0;
                    static constexpr uint8_t P_GYRO_CAL_ENABLE         = 1;
                    static constexpr uint8_t P_MAG_CAL_ENABLE          = 2;
                    static constexpr uint8_t P_SUBCOMMAND              = 3;
                    static constexpr uint8_t P_PLANAR_ACCEL_CAL_ENABLE = 4;
                    static constexpr uint8_t P_ON_TABLE_CAL_ENABLE     = 5;

                    static constexpr uint8_t R_STATUS                  = 5;
                    static constexpr uint8_t R_ACCEL_CAL_ENABLE        = 6;
                    static constexpr uint8_t R_GYRO_CAL_ENABLE         = 7;
                    static constexpr uint8_t R_MAG_CAL_ENABLE          = 8;
                    static constexpr uint8_t R_PLANAR_ACCEL_CAL_ENABLE = 9;
                    static constexpr uint8_t R_ON_TABLE_CAL_ENABLE     = 10;
                }
            }
        }
    }
}

/*
Send --FEATURE COMMAND-- Data to the BNO086 in the following Format:

    --Header-------------------
    byte[ 0] = Packet Length LSB
    byte[ 1] = Packet Length MSB
    byte[ 2] = Channel Number
    byte[ 3] = Sequence Number 
    --Header-------------------
    byte[ 4] = Feature Command
    byte[ 5] = Report ID 
    byte[ 6] = 0x00 (Feature Flags)
    byte[ 7] = 0x00 (Change Sensitivity LSB)
    byte[ 8] = 0x00 (Change Sensitivity MSB)
    byte[ 9] = Report Interval in microseconds LSB 
    byte[10] = Report Inverval
    byte[11] = Report Interval
    byte[12] = Report Interval in microseconds MSB
    byte[13] = 0x00 (Batch Interval LSB)
    byte[14] = 0x00 (Batch Interval)
    byte[15] = 0x00 (Batch Interval)
    byte[16] = 0x00 (Batch Interval MSB)
    byte[17] = Sensor Spesific Config LSB
    byte[18] = Sensor Spesific Config
    byte[19] = Sensor Spesific Config 
    byte[20] = Sensor Spesific Config MSB

    -The sequence number increments with each packet sent and has a different counter for every channel 


Recieve Data Packets from the BNO086 in the following Format:

    packet_header[ 0] = Data Length LSB
    packet_header[ 1] = Data Length MSB
    packet_header[ 2] = Channel 
    packet_header[ 3] = Sequence Number

    packet_buffer[ 0] = Timestamp Report ID 0xFB
    packet_buffer[ 1] = Time since reading was taken in 100 microseconds LSB
    packet_buffer[ 2] = Time since reading was taken in 100 microseconds
    packet_buffer[ 3] = Time since reading was taken in 100 microseconds
    packet_buffer[ 4] = Time since reading was taken in 100 microseconds MSB
    packet_buffer[ 5] = ReportID of the used Sensor
    packet_buffer[ 6] = Sequence Number
    packet_buffer[ 7] = Accuracy Status (0 = Unreliable, 1 = Accuracy Low, 2 = Medium, 3 = High)
    packet_buffer[ 8] = Delay
    packet_buffer[ 9] = Quaternion x | Accelerometer x | Gyroscope x | LSB | Sensor Data Start
    packet_buffer[10] = Quaternion x | Accelerometer x | Gyroscope x | MSB
    packet_buffer[11] = Quaternion y | Accelerometer y | Gyroscope y | LSB
    packet_buffer[12] = Quaternion y | Accelerometer y | Gyroscope y | MSB
    packet_buffer[13] = Quaternion z | Accelerometer z | Gyroscope z | LSB
    packet_buffer[14] = Quaternion z | Accelerometer z | Gyroscope z | MSB
    packet_buffer[15] = Quaternion w | Gyroscope Temperature         | LSB
    packet_buffer[16] = Quaternion w | Gyroscope Temperature         | MSB
    packet_buffer[17] = Estimated Accuracy LSB
    packet_buffer[18] = Estimated Accuracy MSB

*/


/*

bno_constants

    driver_config
    shtp
    i2c
    channel
    control_type
    control_type_cargo_length
    command
    initialization
    device_state
    frs_config
    frs_write_status
    frs_read_status
    advertisement_packet
    bitmask_config
    bitmap_feature_command_feature_flags
    scale_factor
    data_accuracy
    sensor
    stability_classifier
    batching
    report_data_offset
    config_data_offset
        frs_write
        frs_read
        feature_response
        product_id_response
        command
            metadata
            tare 
            initialize

*/