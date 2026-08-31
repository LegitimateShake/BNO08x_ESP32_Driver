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

        bno_err_t shtp_read_packet_internal(shtp_packet_t& rxPacket, size_t bytes_in_buffer, size_t bytes_to_read, size_t& remaining_bytes);

    public:
        
        shtp();

        ~shtp();

        /**
         * @brief initializes SHTP and I2C. Must be called before use 
         * @param i2c_address The I2C address of the device
         * @param i2c_clock_speed_device The I2C communication speed
         * @param i2c_bus pointer to the handle of the I2C master bus that the device is connected to
         * @returns bno_err_t error code. OK on success
         */
        bno_err_t begin(uint16_t i2c_address, uint32_t i2c_clock_speed_device, i2c_master_bus_handle_t* i2c_bus);

        /**
         * @brief Check if the SHTP class was initialized
         */
        bool is_initialized();

        /**
         * @brief Read only the packet length of the next packet in the buffer
         * @param rxPacket Reference to the packet struct that the length is written to
         * @return length of the packet that can be read from the device in bytes (Header + Cargo)
         */
        size_t shtp_get_packet_length(shtp_packet_t& rxPacket);

        /**
         * @brief Try to read a full packet from the device.
         * @param rxPacket The packet struct that should be filled
         * @param bytes_to_read The amount of bytes to read from the device (Header + Cargo) 
         * @param remaining_bytes If the amount of bytes could not be read in a single transfer the remaining bytes will be written into this variable
         * @return bno_err_t error code. OK on success
         */
        bno_err_t shtp_read_packet(shtp_packet_t& rxPacket, size_t bytes_to_read, size_t& remaining_bytes);

        /**
         * @brief Must be called if the previous packet read could not be finished in one transfer
         * @param rxPacket The packet struct that should be filled
         * @param bytes_to_read The amount of bytes to read from the device (Header + Cargo) 
         * @param remaining_bytes If the amount of bytes could not be read in a single transfer the remaining bytes will be written into this variable
         * @return bno_err_t error code. OK on success
         */
        bno_err_t shtp_read_remaining_packet(shtp_packet_t& rxPacket, size_t bytes_to_read, size_t& remaining_bytes);

        /**
         * @brief Send a packet to the device
         * @param txPacket The packet that should be sent
         * @return bno_err_t error code. OK on success
         */
        bno_err_t shtp_send_packet(const shtp_packet_t& txPacket);
};