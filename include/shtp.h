#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/i2c_master.h>
#include <cstdint>
#include <cstring>

#include <bno_types.h>
#include <bno_constants.h>

class shtp {

    private:

        i2c_master_bus_handle_t* _i2c_bus;
        i2c_device_config_t      _sensor_config;
        i2c_master_dev_handle_t  _sensor_handle;
        
        SemaphoreHandle_t _mutex_I2C = nullptr;

        uint8_t _i2c_tx_buffer[bno_constants::i2c::MAX_I2C_TX_BUFFER];
        uint8_t _i2c_rx_buffer[bno_constants::i2c::MAX_I2C_RX_BUFFER];

        bool _initialized = false;

        /**
         * @brief Checks if the desired bytes to read fit into the input buffer
         * @returns The amount of bytes that can be stored in one transfer
         */
        size_t limit_read_to_buffer_size(size_t bytes_to_read);

    public:
        
        shtp();

        ~shtp();

        /**
         * @brief initializes SHTP and I2C. Must be called before use 
         * @param i2c_address The I2C address of the device
         * @param i2c_clock_speed_device The I2C communication speed
         * @param i2c_bus pointer to the handle of the I2C master bus that the device is connected to
         * @returns bno_err_t status code. `bno_err_t::OK` on success
         */
        bno_err_t begin(uint16_t i2c_address, uint32_t i2c_clock_speed_device, i2c_master_bus_handle_t* i2c_bus);

        /**
         * @brief Check if the SHTP class was initialized
         */
        bool is_initialized();

        /**
         * @brief Read only the header and packet length of the next packet in the buffer
         * @param rxPacket Reference to the packet struct that the header is written to. This packet will be prepared for a new transfer, meaning it will be reset
         * @param packet_length Reference to a variable that the packet length should be written to
         * @return bno_err_t status code. `bno_err_t::OK` on success
         */
        bno_err_t shtp_read_header_get_packet_length(shtp_packet_t& rxPacket, size_t& packet_length);

        /**
         * @brief Try to read a full packet from the device. If the I2C input buffer is not large enough, only a partial read will be performed
         * @param rxPacket The packet struct that should be filled
         * @param bytes_to_read The amount of bytes to read from the device (Header + Cargo). Can be estimated or found by a prior read of the header.
         * @param bytes_in_buffer The amount of bytes that are already in the packet buffer. The function updates this value to the new amount after the performed read
         * @param remaining_bytes If the amount of bytes could not be read in a single transfer the remaining bytes will be written into this variable.
         *                        This can the be used as the bytes_to_read parameter in the next function call
         * @return bno_err_t status code. `bno_err_t::OK` on success
         */
        bno_err_t shtp_read_packet(shtp_packet_t& rxPacket, size_t bytes_to_read, size_t& bytes_in_buffer, size_t& remaining_bytes);

        /**
         * @brief Send a packet to the device
         * @param txPacket The packet that should be sent
         * @return bno_err_t status code. `bno_err_t::OK` on success
         */
        bno_err_t shtp_send_packet(const shtp_packet_t& txPacket);
};