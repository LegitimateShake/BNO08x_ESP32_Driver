#pragma once
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <freertos/FreeRTOS.h> 
#include <freertos/semphr.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include <bno_types.h>
#include <bno_constants.h>
#include <bno_parser.h>
#include <shtp.h>

class SH2 {

    private:

        static constexpr char TAG[] = "SH2";
        
        bno_parser _parser;
        bno_data_t _data;
        data_access_sync _data_access;

        shtp* _shtp = nullptr;

        shtp_packet_t _tx_packet;
        shtp_packet_t _rx_packet;
        
        uint8_t _channel_seq_num[5] = {0};
        uint8_t _command_seq_num = 0;

        SemaphoreHandle_t _mutexWriteMessage    = nullptr;
        TaskHandle_t      _bno_read_task_handle = nullptr;

        gpio_num_t _int_pin;
        gpio_num_t _rst_pin;
        bool _initialized = false;

        /**
         * @brief Interrupt that is asserted by the BNO086 when it needs attention
         */
        static void IRAM_ATTR ISR_BNO086_event(void* arg);

        /**
         * @brief Wrapper that calls packet_read_task() member function
         */
        static void task_packet_read_wrapper(void* arg);

        /**
         * @brief Task that is reading and processing data from the BNO086
         */
        void task_packet_read();

        /**
         * @brief Configures the header for a packet
         * @param channel The channel the packet should be delivered on
         * @param packet_size Amount of bytes included in the packet (header + cargo)
         */
        void build_header(uint8_t channel, uint16_t packet_size);

        bno_err_t send_FRS_write_request(uint16_t wordLength, uint16_t frs_type);

        bno_err_t send_FRS_write_data_request(uint32_t word_0, uint32_t word_1, uint16_t offset);

        bno_err_t send_FRS_read_request(uint16_t frs_type);

        /**
         * @brief Sends a command request to the sensor
         * @param command The command that should be executed
         * @param parameters Array holding the command parameters
         * @param parameter_amount The amount of parameters in the array
         * 
         */
        bno_err_t send_command_request(uint8_t command, uint8_t*parameters = nullptr, size_t parameter_amount = 0);

        bno_err_t init_frs_write(uint16_t frs_type, uint16_t len);

        bno_err_t write_frs_words(uint32_t word_0, uint32_t word_1, uint16_t offset);

        bno_err_t finish_frs_write();

    public: 

~SH2();

/**
 * @brief Initialization function. Must be called first to initialize the SH2 object
 * @param transport_layer pointer to the used SHTP implementation
 * @param H_INT GPIO that is connected to the sensors interrupt pin
 * @param RST GPIO that is connected to the sensors reset pin
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t begin(shtp* transport_layer, gpio_num_t H_INT = GPIO_NUM_NC, gpio_num_t RST = GPIO_NUM_NC);


/**
 * @brief Sends an instruction on the executable channel
 * @param command One byte, containing the executable command
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t send_executable_command(uint8_t command);


/**
 * @brief Request the product ID from the sensor
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t send_product_id_request();


/**
 * @brief Send a tare command to the bno08x
 * @param sensor The quaternion output that should be tared
 * @param axis Select if only the z axis or all three axis should be tared
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t send_command_tare_now(bno_tare_sensor_t sensor, bno_tare_axis_t axis);


/**
 * @brief Configure a reorientation quaternion that will be applied to each quaternion output
 * @param w quaternion real component
 * @param x quaternion x component
 * @param y quaternion y component
 * @param z quaternion z component
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t send_command_set_reorientation(float w, float x, float y, float z);


/**
 * @brief Request the current motion engine calibration configuration from the sensor
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t send_command_get_me_calibration();


/**
 * @brief Configures which parts of the motion engine calibration system should be turned on/off
 * @param config The configuration that should be sent to the sensor. 
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t send_command_configure_me_calibration(const me_calibration_config_t& config);


/**
 * @brief Request information on what kind of oscillator the sensor is using 
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t send_command_get_oscillator_type();


/**
 * @brief Set a feature command. This enables and disables sensor outputs
 * @param config Filled configuration struct for the sensor
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t set_feature_command(const bno_sensor_config_t& config);


/**
 * @brief Request the current configuration of a given sensor
 * @param sensor_id Id of the sensor 
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t get_feature_request(uint8_t sensor_id);


/**
 * @brief Write into the flash record system of the sensor
 * @param frs_type The type of record that should be written to
 * @param src Pointer to an array that is holding the words that should be written to the frs
 * @param word_amount The amount of words in the src array
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t write_frs_record(uint16_t frs_type, const uint32_t*src, uint16_t word_amount);


/**
 * @brief Reads an frs record from the sensor
 * @param frs_type The type of record that should be read
 * @param dest Array in which the read data should be stored
 * @param buffer_size How many uint32_t the supplied buffer can hold
 * @param words_stored Reference to a variable in which the amount of stored words will be written
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t read_frs_record(uint16_t frs_type, uint32_t*dest, size_t buffer_size, size_t& words_stored);


/**
 * @brief Returns a reference to the data struct. 
 * @brief This struct should only be accessed through data_access_sync function calls to avoid race conditions
 * @returns reference to the data struct
 */
inline bno_data_t& storage() { return _data; }


/**
 * @brief Disables the sensor by pulling the RST pin low
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t disable_sensor() {
    if(gpio_set_level(_rst_pin, 0) == ESP_OK) return bno_err_t::OK;
    else return bno_err_t::SET_PIN_LEVEL_FAILED;
}


/**
 * @brief Enables the sensor by pulling the RST pin high
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t enable_sensor() {
    if(gpio_set_level(_rst_pin, 1) == ESP_OK) return bno_err_t::OK;
    else return bno_err_t::SET_PIN_LEVEL_FAILED;
}


/**
 * @brief Checks a bitmap for new data and copies the data into the supplied struct. 
 * @brief If data is available, the corresponding bit of the bitmap is cleared
 * @param sensor_id id of the sensor that should be waited for
 * @param src Reference to the internal SH2 storage place of the data. Can be accessed through storage()
 * @param dest The struct the data should be copied to
 * @param ticks_to_timeout The amount of ticks before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
template<typename T>
bno_err_t get_sensor_data_packet(uint8_t sensor_id, const T& src, T& dest, TickType_t ticks_to_timeout) 
{   
    uint64_t mask = (uint64_t)1 << sensor_id;

    if(_data_access.sensor_data().wait_for_mask_and_clear_and_lock(mask, ticks_to_timeout)) {

        dest = src;
        _data_access.unlock();
        return bno_err_t::OK;
    }
    return bno_err_t::BNO_TIMEOUT;
}


/**
 * @brief Checks a bitmap for new config packets and copies the data into the supplied struct. 
 * @brief If data is available, all bits of the mask will be cleared
 * @param mask mask of bits that should be checked. This operates at an logic OR, meaning one set bit is enough
 * @param src Reference to the internal SH2 storage place of the data. Can be accessed through storage()
 * @param dest The struct the data should be copied to
 * @param ticks_to_timeout The amount of ticks before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
template<typename T>
bno_err_t get_config_packet(uint64_t mask, const T& src, T& dest, TickType_t ticks_to_timeout) 
{   
    if(_data_access.sensor_config().wait_for_mask_and_clear_and_lock(mask, ticks_to_timeout)) {

        dest = src;
        _data_access.unlock();
        return bno_err_t::OK;
    }
    return bno_err_t::BNO_TIMEOUT;
}


/**
 * @brief Clears all set bits of the mask from the sensor config bitmap
 */
void clear_config_mask(uint64_t mask)
{
    _data_access.sensor_config().clear_mask(mask);
}


/**
 * @brief Clears all set bits of the mask from the sensor data bitmap
 */
void clear_sensor_data_mask(uint64_t mask)
{
    _data_access.sensor_data().clear_mask(mask);
}


};