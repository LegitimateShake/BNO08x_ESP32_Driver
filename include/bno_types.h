#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <bno_constants.h>

/**
 * @brief Possible error codes of the BNO08x
 * @brief `bno_err_t::OK` is the default return value on success
 */
enum class bno_err_t : uint16_t {

    OK                                    = 100,
    INVALID_INPUT                         = 101,
    MEMORY_ALLOCATION_FAILED              = 102,
    PIN_CONFIG_FAILED                     = 103,
    INT_CONFIG_FAILED                     = 104,
    FAILED_TO_CREATE_TASK                 = 105,
    FAILED_TO_CREATE_MUTEX                = 106,
    FAILED_TO_CREATE_I2C_DEVICE           = 107,
    SET_PIN_LEVEL_FAILED                  = 108,
    SHTP_READ_FAILED                      = 200,
    SHTP_WRITE_FAILED                     = 201,
    SHTP_RX_BUFFER_OVERFLOW               = 202,
    SHTP_TX_PACKET_TOO_LARGE              = 203,
    SHTP_TX_PACKET_TOO_SMALL              = 204,
    SHTP_OBJECT_NOT_INITIALIZED           = 205,
    PARSER_OBJECT_NOT_INITIALIZED         = 300,
    PARSER_SYNC_OBJECT_NOT_INITIALIZED    = 301,
    SH2_INVALID_CHANNEL                   = 400,
    SH2_INVALID_REPORT_ID                 = 401,
    SH2_INVALID_REPORT_LENGTH             = 402,
    SH2_INVALID_COMMAND_ID                = 403,
    SH2_INVALID_CONTROL_ID                = 404,
    BNO_INVALID_SENSOR_ID                 = 500,
    BNO_TIMEOUT                           = 501,
    BNO_CALIBRATION_COULD_NOT_BE_STARTED  = 502,
    BNO_CALIBRATION_ERROR                 = 503,
    FRS_RECORD_EMPTY                      = 600,
    FRS_NO_DATA_RECEIVED                  = 601,
    FRS_TRANSFER_ONGOING                  = 602,
    FRS_WRONG_TYPE                        = 603,
    FRS_READ_ERROR                        = 604,
    FRS_INTERNAL_BUFFER_OVERFLOW          = 605,
    FRS_WRITE_INITIALIZATION_FAILED       = 606,
    FRS_WRITE_ERROR                       = 607,
    FRS_WRITE_COULD_NOT_BE_FINISHED       = 608
};


/**
 * @brief Function checks the return code of a BNO08x function call
 * @brief Loggs the error code on error
 * @brief Returns if the status code is `bno_err_t::OK`
 * @param tag Information that should be logged in the case of an error
 * @param sc Status Code of any function call
 * @param pause_program_on_error Block the program indefinetly when the status code does not equal `bno_err_t::OK`
 */
static inline void bno_check_sc(const char* tag, bno_err_t sc, bool pause_program_on_error = true) {
    if (sc == bno_err_t::OK) {return;}
    ESP_LOGE("BNO08x", "[%s] returned invalid status code: %u", tag, sc);
    while (pause_program_on_error)
        vTaskDelay(pdMS_TO_TICKS(10000));
}


/**
 * @brief IDs of each sensor output of the BNO08x
 * @brief Some outputs are only supported when the BNO08x is connected to environment sensors
 */
enum class bno_sensor_id_t : uint8_t {

    ACCELEROMETER                         = bno_constants::sensor::ACCELEROMETER.id,                       
    GYROSCOPE                             = bno_constants::sensor::GYROSCOPE.id,                   
    MAGNETIC_FIELD                        = bno_constants::sensor::MAGNETIC_FIELD.id,                        
    LINEAR_ACCELERATION                   = bno_constants::sensor::LINEAR_ACCELERATION.id,                  
    ROTATION_VECTOR                       = bno_constants::sensor::ROTATION_VECTOR.id,                       
    GRAVITY                               = bno_constants::sensor::GRAVITY.id,                       
    UNCALIBRATED_GYRO                     = bno_constants::sensor::UNCALIBRATED_GYRO.id,                
    GAME_ROTATION_VECTOR                  = bno_constants::sensor::GAME_ROTATION_VECTOR.id,                
    GEOMAGNETIC_ROTATION_VECTOR           = bno_constants::sensor::GEOMAGNETIC_ROTATION_VECTOR.id,          
    PRESSURE                              = bno_constants::sensor::PRESSURE.id,               
    AMBIENT_LIGHT                         = bno_constants::sensor::AMBIENT_LIGHT.id,             
    HUMIDITY                              = bno_constants::sensor::HUMIDITY.id,             
    PROXIMITY                             = bno_constants::sensor::PROXIMITY.id,              
    TEMPERATURE                           = bno_constants::sensor::TEMPERATURE.id,             
    UNCALIBRATED_MAGNETIC_FIELD           = bno_constants::sensor::UNCALIBRATED_MAGNETIC_FIELD.id,          
    TAP_DETECTOR                          = bno_constants::sensor::TAP_DETECTOR.id,             
    STEP_COUNTER                          = bno_constants::sensor::STEP_COUNTER.id,            
    SIGNIFICANT_MOTION_DETECTOR           = bno_constants::sensor::SIGNIFICANT_MOTION_DETECTOR.id,         
    STABILITY_CLASSIFIER                  = bno_constants::sensor::STABILITY_CLASSIFIER.id,              
    RAW_ACCELEROMETER                     = bno_constants::sensor::RAW_ACCELEROMETER.id,             
    RAW_GYROSCOPE                         = bno_constants::sensor::RAW_GYROSCOPE.id,          
    RAW_MAGNETOMETER                      = bno_constants::sensor::RAW_MAGNETOMETER.id,           
    SHAKE_DETECTOR                        = bno_constants::sensor::SHAKE_DETECTOR.id,           
    FLIP_DETECTOR                         = bno_constants::sensor::FLIP_DETECTOR.id,           
    PICKUP_DETECTOR                       = bno_constants::sensor::PICKUP_DETECTOR.id,           
    STABILITY_DETECTOR                    = bno_constants::sensor::STABILITY_DETECTOR.id,           
    PERSONAL_ACTIVITY_CLASSIFIER          = bno_constants::sensor::PERSONAL_ACTIVITY_CLASSIFIER.id,         
    SLEEP_DETECTOR                        = bno_constants::sensor::SLEEP_DETECTOR.id,           
    TILT_DETECTOR                         = bno_constants::sensor::TILT_DETECTOR.id,           
    POCKET_DETECTOR                       = bno_constants::sensor::POCKET_DETECTOR.id,           
    CIRCLE_DETECTOR                       = bno_constants::sensor::CIRCLE_DETECTOR.id,           
    HEART_RATE_MONITOR                    = bno_constants::sensor::HEART_RATE_MONITOR.id,           
    AR_VR_STABILIZED_ROTATION_VECTOR      = bno_constants::sensor::AR_VR_STABILIZED_ROTATION_VECTOR.id,    
    AR_VR_STABILIZED_GAME_ROTATION_VECTOR = bno_constants::sensor::AR_VR_STABILIZED_GAME_ROTATION_VECTOR.id,
    GYRO_INTEGRATED_ROTATION_VECTOR       = bno_constants::sensor::GYRO_INTEGRATED_ROTATION_VECTOR.id 
};


/**
 * @brief Accuracy estimate for the returned data. This is generated by the BNO08x
 */
enum class bno_sensor_accuracy_t : uint8_t {

    UNRELIABLE = bno_constants::data::sensor::accuracy::UNRELIABLE,
    LOW        = bno_constants::data::sensor::accuracy::LOW,
    MEDIUM     = bno_constants::data::sensor::accuracy::MEDIUM,
    HIGH       = bno_constants::data::sensor::accuracy::HIGH,
    UNKNOWN    = bno_constants::data::sensor::accuracy::UNKNOWN
};


/**
 * @brief The physical state of the senor.
 */
enum class bno_stablitity_state_t : uint8_t {

    UNKNOWN    = bno_constants::data::sensor::stability_classifier::UNKNOWN,
    ON_TABLE   = bno_constants::data::sensor::stability_classifier::ON_TABLE,
    STATIONARY = bno_constants::data::sensor::stability_classifier::STATIONARY,
    STABLE     = bno_constants::data::sensor::stability_classifier::STABLE,
    IN_MOTION  = bno_constants::data::sensor::stability_classifier::IN_MOTION
};


enum class bno_intialization_state_t : uint8_t {

    SUCCESSFUL       = bno_constants::data::command::initialization_state::SUCCESSFUL,
    OPERATION_FAILED = bno_constants::data::command::initialization_state::OPERATION_FAILED,
    UNKNOWN          = bno_constants::data::command::initialization_state::UNKNOWN
};


enum class bno_device_state_t : uint8_t {

    RESERVED = bno_constants::executable_command::RESERVED,
    RESET    = bno_constants::executable_command::RESET,
    ON       = bno_constants::executable_command::ON,
    SLEEP    = bno_constants::executable_command::SLEEP
};


enum class bno_tare_axis_t : uint8_t {

    TARE_X_Y_Z = bno_constants::data::command::tare::axis::X_Y_Z,
    TARE_Z     = bno_constants::data::command::tare::axis::Z     
};


enum class bno_tare_sensor_t : uint8_t {

    ROTATION_VECTOR                       = bno_constants::data::command::tare::sensor::ROTATION_VECTOR,
    GAME_ROTATION_VECTOR                  = bno_constants::data::command::tare::sensor::GAME_ROTATION_VECTOR,
    GEOMAGNETIC_ROTATION_VECTOR           = bno_constants::data::command::tare::sensor::GEOMAGNETIC_ROTATION_VECTOR,
    GYRO_INTEGRATED_ROTATION_VECTOR       = bno_constants::data::command::tare::sensor::GYRO_INTEGRATED_ROTATION_VECTOR,
    AR_VR_STABILIZED_ROTATION_VECTOR      = bno_constants::data::command::tare::sensor::AR_VR_STABILIZED_ROTATION_VECTOR,
    AR_VR_STABILIZED_GAME_ROTATION_VECTOR = bno_constants::data::command::tare::sensor::AR_VR_STABILIZED_GAME_ROTATION_VECTOR
};


struct bno_config_t {

    uint16_t i2c_address                = bno_constants::i2c::DEFAULT_ADDRESS;
    uint8_t rst                         = bno_constants::driver_config::BNO_PIN_UNDEFINED;
    uint8_t h_int                       = bno_constants::driver_config::BNO_PIN_UNDEFINED;
    i2c_master_bus_handle_t* bus_handle = nullptr;
    uint32_t i2c_frequency              = bno_constants::i2c::DEFAULT_CLOCK_SPEED;
};


struct bno_sensor_config_t {

    uint8_t  sensor_id              = bno_constants::driver_config::BNO_SENSOR_UNDEFINED;
    uint8_t  feature_flags          = 0x00;
    uint16_t change_sensitivity     = 0x00;
    uint32_t report_interval        = 0x00;
    uint32_t sensor_spesific_config = 0x00;
    uint32_t batch_interval         = 0x00;
};


struct shtp_header_t {

    uint8_t lengthLSB      = 0;
    uint8_t lengthMSB      = 0;
    uint8_t channel        = 0;
    uint8_t sequenceNumber = 0;
};


struct shtp_packet_t {

    shtp_header_t header;
    uint8_t data[bno_constants::shtp::SHTP_PACKET_BUFFER_SIZE] = {0};
    size_t size   = 0; //Header + Cargo 
    bool overflow = false;
};


struct sensor_metadata_t {

    bno_sensor_accuracy_t accuracy    = bno_sensor_accuracy_t::UNKNOWN;
    uint8_t sequence_number           = 0;
    uint16_t data_delay               = 0;    
    int32_t  base_timestamp_reference = 0;
};


struct quaternion_t {

    sensor_metadata_t metadata;
    
    float w = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    float heading_accuracy_estimate = 0.0f;
};

struct accel_gyro_mag_t {

    sensor_metadata_t metadata;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct raw_accel_gyro_mag_t {

    sensor_metadata_t metadata;

    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    int16_t  temperature = 0;
    uint32_t timestamp   = 0; // microseconds
};

struct uncalibrated_gyro_mag_t {

    sensor_metadata_t metadata;

    float x      = 0.0f;
    float y      = 0.0f;
    float z      = 0.0f; 

    float bias_x = 0.0f;
    float bias_y = 0.0f;
    float bias_z = 0.0f;
};

struct gyro_integrated_rot_vec_t {

    float w     = 0.0f;
    float x     = 0.0f;
    float y     = 0.0f;
    float z     = 0.0f;

    float vel_x = 0.0f;
    float vel_y = 0.0f;
    float vel_z = 0.0f;
};

struct env_sensor_t {

    sensor_metadata_t metadata;
    float data;
};

struct stability_classifier_t {
    
    sensor_metadata_t metadata;
    bno_stablitity_state_t state = bno_stablitity_state_t::UNKNOWN;
};

struct stability_detector_t {

    sensor_metadata_t metadata;
    bno_stablitity_state_t state = bno_stablitity_state_t::UNKNOWN;
};

struct significant_motion_detector_t {

    sensor_metadata_t metadata;
    bool significant_motion = false;
};

struct tap_detector_t {

    sensor_metadata_t metadata;

    bool tap            = false;
    bool doubleTap      = false;

    int pos_tap_x       = 0; 
    int pos_tap_y       = 0; 
    int pos_tap_z       = 0;

    int neg_tap_x       = 0;
    int neg_tap_y       = 0; 
    int neg_tap_z       = 0;

    int pos_doubleTap_x = 0; 
    int pos_doubleTap_y = 0; 
    int pos_doubleTap_z = 0;

    int neg_doubleTap_x = 0; 
    int neg_doubleTap_y = 0; 
    int neg_doubleTap_z = 0;
};

struct command_metadata_t {

    uint8_t command_sequence_number  = 0;   // The sequence number of the sent command request
    uint8_t response_sequence_number = 0;   // Increments when more than one response is sent for a single request
};

struct command_initialized_t {

    command_metadata_t metadata;
    bno_intialization_state_t status = bno_intialization_state_t::UNKNOWN;
};

struct bno_command_data_t {

    uint8_t sequence_number = 0;
    command_initialized_t initialized;
};

struct bno_product_id_t {

    uint8_t reset_cause            = 0x00;
    uint8_t software_version_major = 0x00;
    uint8_t software_version_minor = 0x00;
    uint32_t software_part_number  = 0x00;
    uint32_t software_build_number = 0x00;
    uint16_t software_patch_number = 0x00;
};

struct bno_config_data_t {

    bno_sensor_config_t sensor_config;
    bno_product_id_t product_id;
};

struct sh2_advertisement_data_t {

    char shtp_version[bno_constants::advertisement_packet::CHAR_INPUT_BUFFER_SIZE]      = {0};
    char sensorhub_version[bno_constants::advertisement_packet::CHAR_INPUT_BUFFER_SIZE] = {0};

    uint16_t max_transfer_read  = 0;
    uint16_t max_transfer_write = 0;

    bno_device_state_t device_state = bno_device_state_t::RESERVED;
};

struct bno_sensor_data_t {

    significant_motion_detector_t significant_motion_detector;
    stability_classifier_t stablitity_classifier;
    stability_detector_t stability_detector;
    tap_detector_t tap_counter;

    quaternion_t rotation_vector;
    quaternion_t game_rotation_vector;
    quaternion_t geomagnetic_rotation_vector;
    quaternion_t ar_vr_stabilized_rotation_vector;
    quaternion_t ar_vr_stabilized_game_rotation_vector;

    gyro_integrated_rot_vec_t gyro_integrated_rotation_vector;

    accel_gyro_mag_t gravity;
    accel_gyro_mag_t acceleration;
    accel_gyro_mag_t linear_acceleration;

    accel_gyro_mag_t        gyroscope;
    uncalibrated_gyro_mag_t uncalibrated_gyroscope;

    accel_gyro_mag_t        magnetic_field;
    uncalibrated_gyro_mag_t uncalibrated_magnetic_field;

    raw_accel_gyro_mag_t raw_acceleration;
    raw_accel_gyro_mag_t raw_gyroscope;
    raw_accel_gyro_mag_t raw_magnetometer;

    env_sensor_t temperature;
    env_sensor_t pressure;
    env_sensor_t humidity;
};

struct bno_frs_write_response_t {

    uint8_t status_code  = 0;
    uint16_t word_offset = 0;
};

struct bno_frs_read_response_t {

    uint16_t frs_type     = 0;
    uint8_t  status_code  = 0;
    uint32_t words[bno_constants::driver_config::FRS_READ_BUFFER_SIZE] = {0};
    size_t  words_stored  = 0; 
    bool buffer_overflow = false;
    bool transfer_complete = true;
};

struct bno_frs_data_t {

    bno_frs_write_response_t write_response;
    bno_frs_read_response_t  read_response;
};

struct bno_data_t {

    sh2_advertisement_data_t metadata;
    bno_command_data_t       command;
    bno_sensor_data_t        sensor;
    bno_config_data_t        config;
    bno_frs_data_t           frs;
    bno_sensor_config_t      sensor_data;
};

class scoped_mutex_lock {

    public:
        scoped_mutex_lock(SemaphoreHandle_t m) : mutex(m) { xSemaphoreTake(mutex, portMAX_DELAY); }

        ~scoped_mutex_lock() { xSemaphoreGive(mutex); }

    private:
        SemaphoreHandle_t mutex;
};

class sh2_packet_writer {

    private:
        uint8_t* index;

    public:
            sh2_packet_writer(uint8_t* data) : index(data) {}


        void write_uint8_t(uint8_t value) {
            *index++ = value;
        }

        void write_uint16_t(uint16_t value) {

            *index++ = (value >>  0) & 0xFF;
            *index++ = (value >>  8) & 0xFF;
        }

        void write_uint32_t(uint32_t value) {

            *index++ = (value >>  0) & 0xFF;
            *index++ = (value >>  8) & 0xFF;
            *index++ = (value >> 16) & 0xFF;
            *index++ = (value >> 24) & 0xFF;
        }
};

template<typename T>
class bitmask_builder {

    T _mask;

    public:

        constexpr bitmask_builder(T mask = 0) : _mask(mask) {}

        constexpr bitmask_builder& set(uint8_t bit_position) {
            _mask |= ((T)1 << bit_position);
            return *this;
        }

        constexpr bitmask_builder& clear(uint8_t bit_position){
            _mask &= ~((T)1 << bit_position);
            return *this;
        }

        inline constexpr T get() const {
            return _mask;
        }
};

template<typename T>
class bitmap_access {

    private:

        T& _bitmap;
        SemaphoreHandle_t& _mutex;

    public:

        bitmap_access(T& bitmap, SemaphoreHandle_t& mutex) : _bitmap(bitmap), _mutex(mutex) {}

        void set_bit_and_lock( uint8_t bit)
        {
            xSemaphoreTake(_mutex, portMAX_DELAY);
            _bitmap |= ((T)1 << bit);
        }
        
        void set_mask_and_lock(T mask)
        {
            xSemaphoreTake(_mutex, portMAX_DELAY);
            _bitmap |=  mask;
        }

        void clear_bit(uint8_t bit)
        {
            xSemaphoreTake(_mutex, portMAX_DELAY);
            _bitmap &= ~((T)1 << bit);
            xSemaphoreGive(_mutex);
        }

        void clear_mask(T mask)
        {
            xSemaphoreTake(_mutex, portMAX_DELAY);
            _bitmap &= ~mask;
            xSemaphoreGive(_mutex);
        }

        bool wait_for_mask_and_clear_and_lock(T mask, TickType_t ticks_to_timeout)
        {
            TickType_t ticks_now = xTaskGetTickCount();

            while(true) {

            xSemaphoreTake(_mutex, portMAX_DELAY);

            if(_bitmap & mask) {
                _bitmap &= ~mask;
                return true;
            }
            xSemaphoreGive(_mutex);

            TickType_t ticks_elapsed = xTaskGetTickCount() - ticks_now;

            if(ticks_elapsed >= ticks_to_timeout)
                return false;

            TickType_t ticks_remaining = ticks_to_timeout - ticks_elapsed;

            if(ulTaskNotifyTake(pdTRUE, ticks_remaining) == 0)
                    return false;
            }
        }
};

class data_access_sync {

    private:

        // Bitmaps
        uint64_t _sensor_data = 0; // Bits are set according to sensor IDs
        uint64_t _config      = 0; // Bits are set according to bitmap_event namespace constants

        SemaphoreHandle_t _data_access = nullptr;
        TaskHandle_t _task_to_notify   = nullptr;
        bool _initialized              = false;

    public:

        bno_err_t begin() {

            _data_access = xSemaphoreCreateMutex();

            if(_data_access == nullptr)
                return bno_err_t::FAILED_TO_CREATE_MUTEX;

            _task_to_notify = xTaskGetCurrentTaskHandle();
            _initialized    = true;

            return bno_err_t::OK;
        };

        inline bool is_initialized() {
            return _initialized;
        }

        inline void lock() {
            xSemaphoreTake(_data_access, portMAX_DELAY);
        }

        inline void unlock() {
            xSemaphoreGive(_data_access);
        }

        inline void notify_user() {
            xTaskNotifyGive(_task_to_notify);
        }

        bitmap_access<uint64_t> sensor_data() {
            return bitmap_access<uint64_t>(_sensor_data, _data_access);
        }

        bitmap_access<uint64_t> sensor_config() {
            return bitmap_access<uint64_t>(_config, _data_access);
        }
};
