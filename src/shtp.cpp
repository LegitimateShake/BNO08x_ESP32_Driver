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

size_t shtp::shtp_get_packet_length(shtp_packet_t& rxPacket) {

    scoped_mutex_lock lock(_mutex_I2C);

    uint8_t temp_buffer[2];

    if(i2c_master_receive(_sensor_handle, temp_buffer, 2, bno_constants::i2c::I2C_TIMEOUT_MS) != ESP_OK) {
        return 0;
    }
        
    rxPacket.header.lengthLSB = temp_buffer[0];
    rxPacket.header.lengthMSB = temp_buffer[1];

    uint16_t size = (uint16_t)rxPacket.header.lengthMSB << 8 | rxPacket.header.lengthLSB;
    //Bit 15 indicates if the message is a continuation of a previous transfer
    size &= ~(1 << 15);

    return (size_t)size;
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

bno_err_t shtp::shtp_read_packet_internal(shtp_packet_t& rxPacket, size_t bytes_in_buffer, size_t bytes_to_read, size_t& remaining_bytes) {

    namespace I2C = bno_constants::i2c;
    namespace BNO = bno_constants::shtp;

    scoped_mutex_lock lock(_mutex_I2C);

    if(I2C::MAX_I2C_RX_BUFFER < bytes_to_read) {
        
        // The header needs to be read every time, so we need to add it to the remaining bytes            
        remaining_bytes =  bytes_to_read - I2C::MAX_I2C_RX_BUFFER + BNO::SHTP_HEADER_SIZE;
        bytes_to_read   = I2C::MAX_I2C_RX_BUFFER;
    }
    else {
        remaining_bytes = 0;
    }

    if(bytes_to_read < BNO::SHTP_HEADER_SIZE || i2c_master_receive(_sensor_handle, _i2c_rx_buffer, bytes_to_read, I2C::I2C_TIMEOUT_MS) != ESP_OK)
        return bno_err_t::SHTP_READ_FAILED;

    rxPacket.header.channel        = _i2c_rx_buffer[2];
    rxPacket.header.sequenceNumber = _i2c_rx_buffer[3];

    size_t data_bytes_received = bytes_to_read - BNO::SHTP_HEADER_SIZE;
    size_t data_bytes_copied   = 0;

    size_t buf_index = BNO::SHTP_HEADER_SIZE;

    for(size_t i = bytes_in_buffer ; i < bytes_in_buffer + data_bytes_received ; i++) {

        if(i >= BNO::SHTP_PACKET_BUFFER_SIZE) {
            rxPacket.overflow = true;
            break;
        }
        rxPacket.data[i] = _i2c_rx_buffer[buf_index];
        data_bytes_copied++;
        buf_index++;
    }
    rxPacket.size += data_bytes_copied;

    // Add the header size once at the end
    if(remaining_bytes == 0)
        rxPacket.size += BNO::SHTP_HEADER_SIZE;

    if(rxPacket.overflow)
        return bno_err_t::SHTP_RX_BUFFER_OVERFLOW;
    else 
        return bno_err_t::OK;
}

bno_err_t shtp::shtp_read_packet(shtp_packet_t& rxPacket, size_t bytes_to_read, size_t& remaining_bytes) {
    
    rxPacket.size = 0;
    rxPacket.overflow = false;

    return shtp_read_packet_internal(rxPacket, rxPacket.size, bytes_to_read, remaining_bytes);
}

bno_err_t shtp::shtp_read_remaining_packet(shtp_packet_t& rxPacket, size_t bytes_to_read, size_t& remaining_bytes) {

    return shtp_read_packet_internal(rxPacket, rxPacket.size, bytes_to_read, remaining_bytes);
}
    
