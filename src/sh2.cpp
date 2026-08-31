#include "sh2.h"

void IRAM_ATTR SH2::ISR_BNO086_event(void* arg) {

    SH2* sh2_instance = static_cast<SH2*>(arg);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; 

    //Wakes the task that is reading data from the BNO086
    vTaskNotifyGiveFromISR(sh2_instance->_bno_read_task_handle, &xHigherPriorityTaskWoken);

    //Forces a context switch if the currently running task has a lower priority
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void SH2::task_packet_read_wrapper(void* arg) {

    SH2* self = static_cast<SH2*>(arg);
    self->task_packet_read();
}

void SH2::task_packet_read() {

    size_t message_size    = 0;
    size_t remaining_bytes = 0;
    bno_err_t err;

    while(true) {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(message_size == 0) {

            message_size = _shtp->shtp_get_packet_length(_rx_packet);
            continue;
        }
        
        if(remaining_bytes == 0) {
            err = _shtp->shtp_read_packet(_rx_packet, message_size, remaining_bytes);
        }
        else {
            err = _shtp->shtp_read_remaining_packet(_rx_packet, remaining_bytes, remaining_bytes);
        }

        if(remaining_bytes > 0)
            continue;

        if(err != bno_err_t::SHTP_READ_FAILED) {

            _parser.parse(&_rx_packet);
        }
        message_size = 0;
    }
}

SH2::~SH2() {

    if(_rst_pin != bno_constants::driver_config::BNO_PIN_UNDEFINED && _int_pin != bno_constants::driver_config::BNO_PIN_UNDEFINED) {

        gpio_isr_handler_remove(_int_pin);
        gpio_reset_pin(_rst_pin);
        gpio_reset_pin(_int_pin);
    }

    if(_bno_read_task_handle != nullptr) 
        vTaskDelete(_bno_read_task_handle);

    if(_mutexWriteMessage != nullptr)
        vSemaphoreDelete(_mutexWriteMessage);
}

bno_err_t SH2::begin(shtp* transport_layer, uint8_t H_INT, uint8_t RST) {

    bno_err_t sc;

    if(_initialized)
        return bno_err_t::OK;

    if(RST == bno_constants::driver_config::BNO_PIN_UNDEFINED || H_INT == bno_constants::driver_config::BNO_PIN_UNDEFINED ||
       transport_layer == nullptr) 
        return bno_err_t::INVALID_INPUT;
    
    _shtp    = transport_layer;
    _rst_pin = static_cast<gpio_num_t>(RST);
    _int_pin = static_cast<gpio_num_t>(H_INT);

    if(_shtp->is_initialized() == false) {

        ESP_LOGE(TAG, "Initialization failed: SHTP object not initialized");
        return bno_err_t::SHTP_OBJECT_NOT_INITIALIZED;
    }
    
    sc = _data_access.begin();

    if(sc == bno_err_t::OK) {
        ESP_LOGI(TAG, "Data synchronization initialized");
    }
    else {
        ESP_LOGE(TAG, "Initialization failed: Parser could not be initialized. Error Code: %u.", sc);
        return bno_err_t::PARSER_SYNC_OBJECT_NOT_INITIALIZED;
    } 

    sc = _parser.begin(&_data, &_data_access);
    
    if(sc == bno_err_t::OK) {
        ESP_LOGI(TAG, "Parser initialized");
    }
    else {
        ESP_LOGE(TAG, "Initialization failed: Parser could not be initialized. Error Code: %u.", sc);
        return bno_err_t::PARSER_OBJECT_NOT_INITIALIZED;
    }    

    _mutexWriteMessage = xSemaphoreCreateMutex();

    if(_mutexWriteMessage == NULL) 
        return bno_err_t::FAILED_TO_CREATE_MUTEX;

    // Disable the BNO086
    if(gpio_reset_pin(_rst_pin) != ESP_OK ||  gpio_set_direction(_rst_pin, GPIO_MODE_OUTPUT) != ESP_OK || gpio_set_level(_rst_pin, 0) != ESP_OK) {

        ESP_LOGE(TAG, "Initialization failed: Reset pin could not be configured");
        return bno_err_t::PIN_CONFIG_FAILED;
    }
    
    // Create the task that is reading data from the sensor
    if(xTaskCreate(task_packet_read_wrapper, 
                   "sh2_reader", 
                   bno_constants::driver_config::READ_TASK_STACK_SIZE, 
                   this, 
                   bno_constants::driver_config::READ_TASK_PRIORITY, 
                   &_bno_read_task_handle) != pdPASS) 
    {
        vSemaphoreDelete(_mutexWriteMessage);
        ESP_LOGE(TAG, "Initialization failed: Parser task could not be created");
        return bno_err_t::FAILED_TO_CREATE_TASK;
    }
    
    // Configure the interrupt pin
    if(gpio_reset_pin(_int_pin) != ESP_OK ||  gpio_set_direction(_int_pin, GPIO_MODE_INPUT) != ESP_OK || gpio_set_pull_mode(_int_pin, GPIO_PULLUP_ONLY)) {

        ESP_LOGE(TAG, "Initialization failed: Interrupt pin could not be configured");
        vSemaphoreDelete(_mutexWriteMessage);
        vTaskDelete(_bno_read_task_handle);
        return bno_err_t::PIN_CONFIG_FAILED;
    }
    
    // Configure the interrupt
    esp_err_t err = gpio_install_isr_service(0);

    if(err != ESP_OK && err != ESP_ERR_INVALID_STATE) {

        ESP_LOGE(TAG, "Initialization failed: Interrupt GPIO drivers could not be installed");
        vSemaphoreDelete(_mutexWriteMessage);
        vTaskDelete(_bno_read_task_handle);
        return bno_err_t::INT_CONFIG_FAILED;
    }
        
    if( gpio_set_intr_type(_int_pin, GPIO_INTR_NEGEDGE) != ESP_OK || gpio_isr_handler_add(_int_pin, ISR_BNO086_event, this) != ESP_OK ) {

        ESP_LOGE(TAG, "Initialization failed: Interrupt could not be configured");
        vSemaphoreDelete(_mutexWriteMessage);
        vTaskDelete(_bno_read_task_handle);
        return bno_err_t::INT_CONFIG_FAILED;
    }
    
    // Enable the chip after the interrupt was configured
    if(gpio_set_level(_rst_pin, 1) != ESP_OK) {

        vSemaphoreDelete(_mutexWriteMessage);
        gpio_isr_handler_remove(static_cast<gpio_num_t>(_int_pin));
        vTaskDelete(_bno_read_task_handle);
        return bno_err_t::PIN_CONFIG_FAILED;
    }

    _initialized = true;
    ESP_LOGI(TAG, "Initialization complete");

    return bno_err_t::OK;
}

void SH2::build_header(uint8_t channel, uint16_t packet_size) {

    _tx_packet.header.lengthLSB      = packet_size & 0xFF;
    _tx_packet.header.lengthMSB      = packet_size >> 8;
    _tx_packet.header.channel        = channel;
    _tx_packet.header.sequenceNumber = _channel_seq_num[channel]++;

    //Store the amount of bytes that should be transmitted in the packet
    _tx_packet.size = packet_size;

    return;
}

bno_err_t SH2::send_executable_command(uint8_t command) {

    scoped_mutex_lock lock(_mutexWriteMessage);

    build_header(bno_constants::channel::EXECUTABLE, bno_constants::control_type::packet_size::EXECUTABLE);

    sh2_packet_writer out(_tx_packet.data);

    out.write_uint8_t(command);

    return _shtp->shtp_send_packet(_tx_packet);
}

bno_err_t SH2::send_product_id_request() {

    scoped_mutex_lock lock(_mutexWriteMessage);

    build_header(bno_constants::channel::CONTROL, bno_constants::control_type::packet_size::PRODUCT_ID_REQUEST);

    sh2_packet_writer out(_tx_packet.data);

    out.write_uint8_t(bno_constants::control_type::id::PRODUCT_ID_REQUEST);
    out.write_uint8_t(0x00);

    return _shtp->shtp_send_packet(_tx_packet);
}

bno_err_t SH2::send_command_request(uint8_t command, uint8_t*parameters, size_t parameter_amount) {

    if(parameters == nullptr && parameter_amount > 0)
        return bno_err_t::INVALID_INPUT;

    scoped_mutex_lock lock(_mutexWriteMessage);

    build_header(bno_constants::channel::CONTROL, bno_constants::control_type::packet_size::COMMAND_REQUEST);

    sh2_packet_writer out(_tx_packet.data);

    out.write_uint8_t(bno_constants::control_type::id::COMMAND_REQUEST);
    out.write_uint8_t(_command_seq_num++);
    out.write_uint8_t(command);

    for(size_t i = 0 ; i < bno_constants::command::COMMAND_PARAMETER_AMOUNT ; i++) {

        if(i < parameter_amount)
            out.write_uint8_t(parameters[i]);
        else
            out.write_uint8_t(0x00);
    }
    return _shtp->shtp_send_packet(_tx_packet);
}

bno_err_t SH2::set_feature_command(const bno_sensor_config_t& config) {

    scoped_mutex_lock lock(_mutexWriteMessage);

    build_header(bno_constants::channel::CONTROL, bno_constants::control_type::packet_size::SET_FEATURE_COMMAND);

    sh2_packet_writer out(_tx_packet.data);

    out.write_uint8_t(bno_constants::control_type::id::SET_FEATURE_COMMAND);  
    out.write_uint8_t(config.sensor_id);                                        
    out.write_uint8_t(config.feature_flags);                                     
    out.write_uint16_t(config.change_sensitivity);                                                

    out.write_uint32_t(config.report_interval);                               
    out.write_uint32_t(config.batch_interval);                                   
    out.write_uint32_t(config.sensor_spesific_config);                                  
    
    return _shtp->shtp_send_packet(_tx_packet);
}

bno_err_t SH2::get_feature_request(uint8_t sensor_id) {

    scoped_mutex_lock lock(_mutexWriteMessage);

    build_header(bno_constants::channel::CONTROL, bno_constants::control_type::packet_size::GET_FEATURE_REQUEST);

    sh2_packet_writer out(_tx_packet.data);

    out.write_uint8_t(bno_constants::control_type::id::GET_FEATURE_REQUEST);
    out.write_uint8_t(sensor_id);

    return _shtp->shtp_send_packet(_tx_packet);
}

bno_err_t SH2::send_FRS_write_request(uint16_t wordLength, uint16_t frs_type) {

    scoped_mutex_lock lock(_mutexWriteMessage);

    build_header(bno_constants::channel::CONTROL, bno_constants::control_type::packet_size::FRS_WRITE_REQUEST);

    sh2_packet_writer out(_tx_packet.data);

    out.write_uint8_t(bno_constants::control_type::id::FRS_WRITE_REQUEST);
    out.write_uint8_t(0x00);
    out.write_uint16_t(wordLength);
    out.write_uint16_t(frs_type);

    return _shtp->shtp_send_packet(_tx_packet);
}

bno_err_t SH2::send_FRS_write_data_request(uint32_t word_0, uint32_t word_1, uint16_t offset) {

    scoped_mutex_lock lock(_mutexWriteMessage);

    build_header(bno_constants::channel::CONTROL, bno_constants::control_type::packet_size::FRS_WRITE_DATA_REQUEST);

    sh2_packet_writer out(_tx_packet.data);

    out.write_uint8_t(bno_constants::control_type::id::FRS_WRITE_DATA_REQUEST);
    out.write_uint8_t(0x00);
    out.write_uint16_t(offset);

    out.write_uint32_t(word_0);
    out.write_uint32_t(word_1);

    return _shtp->shtp_send_packet(_tx_packet);
}   

bno_err_t SH2::send_FRS_read_request(uint16_t frs_type) {

    scoped_mutex_lock lock(_mutexWriteMessage);

    build_header(bno_constants::channel::CONTROL, bno_constants::control_type::packet_size::FRS_READ_REQUEST);

    sh2_packet_writer out(_tx_packet.data);

    out.write_uint8_t(bno_constants::control_type::id::FRS_READ_REQUEST);
    out.write_uint8_t(0x00);
    out.write_uint8_t(0x00);
    out.write_uint8_t(0x00);
    out.write_uint16_t(frs_type);
    out.write_uint16_t(0x00);

    return _shtp->shtp_send_packet(_tx_packet);
}

bno_err_t SH2::init_frs_write(uint16_t frs_type, uint16_t len) {

    namespace mask = bno_constants::bitmask_config;
    bno_err_t sc;
    
    clear_config_mask(mask::FRS_WRITE_RESPONSE);

    sc = send_FRS_write_request(len, frs_type);
    if(sc != bno_err_t::OK)
        return sc;
    
    bno_frs_write_response_t reply;

    sc = get_config_packet(mask::FRS_WRITE_RESPONSE, _data.frs.write_response, reply, bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT);
    if(sc != bno_err_t::OK)
        return sc;
    
    if(reply.status_code == bno_constants::frs::write_status::WRITE_MODE_ENTERED)
        return bno_err_t::OK;
    else 
        return bno_err_t::FRS_WRITE_INITIALIZATION_FAILED; 
}

bno_err_t SH2::write_frs_words(uint32_t word_0, uint32_t word_1, uint16_t offset) {

    namespace mask = bno_constants::bitmask_config;
    bno_err_t sc;

    sc = send_FRS_write_data_request(word_0, word_1, offset);
    if(sc != bno_err_t::OK)
        return sc;
    
    bno_frs_write_response_t reply;
    
    sc = get_config_packet(mask::FRS_WRITE_RESPONSE, _data.frs.write_response, reply, bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT);
    if(sc != bno_err_t::OK)
        return sc;
    
    if(reply.status_code == bno_constants::frs::write_status::WORDS_RECEIVED)
        return bno_err_t::OK;
    else 
        return bno_err_t::FRS_WRITE_ERROR; 

    return sc;
}

bno_err_t SH2::finish_frs_write() {
    
    namespace frs_sc = bno_constants::frs::write_status;
    namespace mask = bno_constants::bitmask_config;

    bno_err_t sc;
    bno_frs_write_response_t reply;
    
    sc = get_config_packet(mask::FRS_WRITE_RESPONSE, _data.frs.write_response, reply, bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT);
    if(sc != bno_err_t::OK)
        return sc;
    
    if(reply.status_code == frs_sc::RECORD_VALID_AND_WRITE_COMPLETED)
        return bno_err_t::OK;

    if(reply.status_code != frs_sc::RECORD_VALID)
        return bno_err_t::FRS_WRITE_COULD_NOT_BE_FINISHED;

    sc = get_config_packet(mask::FRS_WRITE_RESPONSE, _data.frs.write_response, reply, bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT);
    if(sc != bno_err_t::OK)
        return sc;
    
    if(reply.status_code == frs_sc::RECORD_VALID_AND_WRITE_COMPLETED || reply.status_code == frs_sc::WRITE_COMPLETED)
        return bno_err_t::OK;
    else
        return bno_err_t::FRS_WRITE_COULD_NOT_BE_FINISHED;
}

bno_err_t SH2::write_frs_record(uint16_t frs_type, const uint32_t*src, uint16_t word_amount) {

    bno_err_t sc;

    sc = init_frs_write(frs_type, word_amount);
    
    if(sc != bno_err_t::OK)
        return sc;

    uint16_t transfers = (word_amount + 1u) / 2u;
    uint16_t word_index = 0;
    uint16_t offset = 0;
    uint32_t word_1;
    uint32_t word_2;

    for(int i = 0 ; i < transfers ; i++) {

        word_1 = src[word_index++];
        word_2 = word_index < word_amount ? src[word_index++] : 0x00 ;
        
        sc = write_frs_words(word_1, word_2, offset);
        offset += 2;

        if(sc != bno_err_t::OK)
            return sc;
    }
    return finish_frs_write();
}

bno_err_t SH2::read_frs_record(uint16_t frs_type, uint32_t*dest, size_t buffer_size, size_t& words_stored) {

    namespace mask   = bno_constants::bitmask_config;
    namespace status = bno_constants::frs::read_status;

    if(dest == nullptr)
        return bno_err_t::INVALID_INPUT;

    bno_err_t sc;

    sc = send_FRS_read_request(frs_type);
    if(sc != bno_err_t::OK)
        return sc;

    if(!_data_access.sensor_config().wait_for_mask_and_clear_and_lock(mask::FRS_READ_COMPLETE, bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT))
        return bno_err_t::BNO_TIMEOUT;
    
    bno_frs_read_response_t& response = _data.frs.read_response;

    if(response.frs_type != frs_type) {
        _data_access.unlock();
        return bno_err_t::FRS_WRONG_TYPE;
    }
        
    if(response.status_code == status::RECORD_EMPTY) {
        _data_access.unlock();
        return bno_err_t::FRS_RECORD_EMPTY;
    }
        
    if(response.status_code != status::READ_RECORD_COMPLETED) {
        _data_access.unlock();
        return bno_err_t::FRS_READ_ERROR;
    }
    
    size_t copy_words = std::min(buffer_size, response.words_stored);
    
    if(copy_words > 0)
        std::memcpy(dest, response.words, copy_words * sizeof(uint32_t));

    words_stored = copy_words;

    if(response.buffer_overflow)
        sc = bno_err_t::FRS_INTERNAL_BUFFER_OVERFLOW;
    else
        sc = bno_err_t::OK;

    _data_access.unlock();
    
    return sc;
}

bno_err_t SH2::send_command_tare_now(bno_tare_sensor_t sensor, bno_tare_axis_t axis) {

    namespace offset = bno_constants::data_offset::config::command::tare;

    uint8_t tare_parameters[bno_constants::command::COMMAND_PARAMETER_AMOUNT] = {0x00};
    
    tare_parameters[offset::P_SUBCOMMAND]  = bno_constants::command::subcommand::tare::TARE_NOW;
    tare_parameters[offset::P_BITMAP_AXIS] = (uint8_t)axis;
    tare_parameters[offset::P_SENSOR]      = (uint8_t)sensor;

    return send_command_request(bno_constants::command::TARE.id, tare_parameters, bno_constants::command::COMMAND_PARAMETER_AMOUNT);
}

bno_err_t SH2::send_command_set_reorientation(float w, float x, float y, float z) {

    namespace offset = bno_constants::data_offset::config::command::tare;
    namespace scale  = bno_constants::scale_factor;

    uint8_t tare_parameters[bno_constants::command::COMMAND_PARAMETER_AMOUNT] = {0x00};

    uint16_t _w = uint16_t(int16_t(std::round(w / scale::ROTATION_VECTOR)));
    uint16_t _x = uint16_t(int16_t(std::round(x / scale::ROTATION_VECTOR)));
    uint16_t _y = uint16_t(int16_t(std::round(y / scale::ROTATION_VECTOR)));
    uint16_t _z = uint16_t(int16_t(std::round(z / scale::ROTATION_VECTOR)));

    tare_parameters[offset::P_SUBCOMMAND] = bno_constants::command::subcommand::tare::SET_REORIENTATION;

    sh2_packet_writer params(&tare_parameters[offset::P_REORIENTATION_QUATERNION_START]);

    params.write_uint16_t(_x);
    params.write_uint16_t(_y);
    params.write_uint16_t(_z);
    params.write_uint16_t(_w);

    return send_command_request(bno_constants::command::TARE.id, tare_parameters, bno_constants::command::COMMAND_PARAMETER_AMOUNT);
}