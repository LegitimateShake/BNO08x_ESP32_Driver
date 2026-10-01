#include "shtp.h"

shtp::shtp(){}

shtp::~shtp() {

    i2c_master_bus_rm_device(_sensor_handle);

    if(_mutex_I2C != nullptr)
        vSemaphoreDelete(_mutex_I2C);
}

bno_err_t shtp::begin(uint16_t i2c_address, uint32_t i2c_clock_speed_device, i2c_master_bus_handle_t* i2c_bus) {

    if(_initialized)
        return bno_err_t::OK;

    if(i2c_bus == nullptr)
        return bno_err_t::INVALID_INPUT;

    _mutex_I2C = xSemaphoreCreateMutex();

    if(_mutex_I2C == nullptr) 
        return bno_err_t::FAILED_TO_CREATE_MUTEX;

    _i2c_bus = i2c_bus;

    _sensor_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    _sensor_config.device_address  = i2c_address;
    _sensor_config.scl_speed_hz    = i2c_clock_speed_device;
    _sensor_config.scl_wait_us     = bno_constants::i2c::CLOCK_STRETCH_WAIT_US;

    esp_err_t sc = i2c_master_bus_add_device(*_i2c_bus, &_sensor_config, &_sensor_handle);

    if(sc != ESP_OK) {
        vSemaphoreDelete(_mutex_I2C);
        return bno_err_t::FAILED_TO_CREATE_I2C_DEVICE;
    }
        
    _initialized = true;

    return bno_err_t::OK;
}

bool shtp::is_initialized() {

    return _initialized;
}

bno_err_t shtp::shtp_read_header_get_packet_length(shtp_packet_t& rxPacket, size_t& packet_length) {

    namespace BNO = bno_constants::shtp;

    scoped_mutex_lock lock(_mutex_I2C);

    uint8_t temp_buffer[BNO::SHTP_HEADER_SIZE];

    if(i2c_master_receive(_sensor_handle, temp_buffer, BNO::SHTP_HEADER_SIZE, bno_constants::i2c::I2C_TIMEOUT_MS) != ESP_OK) {
        return bno_err_t::SHTP_READ_FAILED;
    }
        
    rxPacket.header.lengthLSB      = temp_buffer[0];
    rxPacket.header.lengthMSB      = temp_buffer[1];
    rxPacket.header.channel        = temp_buffer[2];
    rxPacket.header.sequenceNumber = temp_buffer[3];

    //Reset the packet
    rxPacket.overflow = false;
    rxPacket.size = 0;

    packet_length = ((uint16_t)rxPacket.header.lengthMSB << 8 | rxPacket.header.lengthLSB) & ~(1 << 15);

    return bno_err_t::OK;
}

bno_err_t shtp::shtp_send_packet(const shtp_packet_t& txPacket) {

    scoped_mutex_lock lock(_mutex_I2C);

    if(txPacket.size > bno_constants::i2c::MAX_I2C_TX_BUFFER)
        return bno_err_t::SHTP_TX_PACKET_TOO_LARGE;
    if(txPacket.size < bno_constants::shtp::SHTP_HEADER_SIZE)
        return bno_err_t::SHTP_TX_PACKET_TOO_SMALL;

    // Header
    _i2c_tx_buffer[0] = txPacket.header.lengthLSB;
    _i2c_tx_buffer[1] = txPacket.header.lengthMSB;
    _i2c_tx_buffer[2] = txPacket.header.channel;
    _i2c_tx_buffer[3] = txPacket.header.sequenceNumber;

    // Cargo
    std::memcpy(&_i2c_tx_buffer[bno_constants::shtp::SHTP_HEADER_SIZE], 
                 txPacket.data, 
                 txPacket.size - bno_constants::shtp::SHTP_HEADER_SIZE);

    if(i2c_master_transmit(_sensor_handle, _i2c_tx_buffer, txPacket.size, bno_constants::i2c::I2C_TIMEOUT_MS) != ESP_OK)
        return bno_err_t::SHTP_WRITE_FAILED;
    
    return bno_err_t::OK;
}

size_t shtp::limit_read_to_buffer_size(size_t bytes_to_read) {

    namespace I2C = bno_constants::i2c;

    if(bytes_to_read > I2C::MAX_I2C_RX_BUFFER) {
        
        return (size_t)I2C::MAX_I2C_RX_BUFFER;
    }
    else {
        return bytes_to_read;
    }
}

bno_err_t shtp::shtp_read_packet(shtp_packet_t& rxPacket, size_t bytes_to_read, size_t& bytes_in_buffer, size_t& remaining_bytes) {

    namespace I2C = bno_constants::i2c;
    namespace BNO = bno_constants::shtp;

    scoped_mutex_lock lock(_mutex_I2C);

    size_t _received_i2c_bytes = limit_read_to_buffer_size(bytes_to_read);
    
    if(_received_i2c_bytes < BNO::SHTP_HEADER_SIZE || i2c_master_receive(_sensor_handle, _i2c_rx_buffer, _received_i2c_bytes, I2C::I2C_TIMEOUT_MS) != ESP_OK)
        return bno_err_t::SHTP_READ_FAILED;
   
    // Check if the packet is a continuation of a previous transfer. Only update Header if that is not the case
    const bool _packet_is_a_continuation = (_i2c_rx_buffer[1] & ((uint8_t)1 << 7)) != 0;

    if(_packet_is_a_continuation == false) {

        rxPacket.header.lengthLSB = _i2c_rx_buffer[0];
        rxPacket.header.lengthMSB = _i2c_rx_buffer[1];
        rxPacket.header.channel   = _i2c_rx_buffer[2];

        rxPacket.overflow = false;
    }
    
    rxPacket.header.sequenceNumber = _i2c_rx_buffer[3];

    size_t _packet_size  = (uint16_t)_i2c_rx_buffer[1] << 8 | _i2c_rx_buffer[0];
           _packet_size &= ~(1 << 15); // Bit is set if the packet is part of a previous transfer

    if(_packet_size < BNO::SHTP_HEADER_SIZE)
        return bno_err_t::SHTP_READ_FAILED;
    
    if(_received_i2c_bytes >= _packet_size) // CASE: We read the correct amount or more than the correct amount of bytes
        remaining_bytes = 0;
    else                                    // CASE: We have not yet read enough bytes
        remaining_bytes = _packet_size - _received_i2c_bytes + BNO::SHTP_HEADER_SIZE; 

    // Here we copy the read data into the rxPacket
    size_t _data_bytes_to_copy = _received_i2c_bytes - BNO::SHTP_HEADER_SIZE;

    if((_data_bytes_to_copy + bytes_in_buffer) > BNO::SHTP_PACKET_BUFFER_SIZE) {

        _data_bytes_to_copy = BNO::SHTP_PACKET_BUFFER_SIZE - bytes_in_buffer;
        rxPacket.overflow = true;
    }

    // If we read too many bytes, the sensor padds the read with zeros
    // Here we check if we read too many bytes and if so, limit the bytes
    // that are copied into the packet to only the data bytes
    size_t _valid_data_bytes = _packet_size - BNO::SHTP_HEADER_SIZE;

    if(_data_bytes_to_copy > _valid_data_bytes)
        _data_bytes_to_copy = _valid_data_bytes;

    std::memcpy(&rxPacket.data[bytes_in_buffer], &_i2c_rx_buffer[BNO::SHTP_HEADER_SIZE], _data_bytes_to_copy);
    
    bytes_in_buffer += _data_bytes_to_copy;
    rxPacket.size    = bytes_in_buffer + BNO::SHTP_HEADER_SIZE;
    
    if(rxPacket.overflow)
        return bno_err_t::SHTP_RX_BUFFER_OVERFLOW;
    else
        return bno_err_t::OK;
}
