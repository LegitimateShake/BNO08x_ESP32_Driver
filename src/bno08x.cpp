#include <bno08x.h>

bno_err_t BNO08x::begin(bno_config_t* sensor_config) {

    if(_initialized)
        return bno_err_t::OK;

    bno_err_t sc;

    if(sensor_config == nullptr) {
        ESP_LOGE(TAG, "Initialization failed: Invalid input argument. Error Code %u", bno_err_t::INVALID_INPUT);
        return bno_err_t::INVALID_INPUT;
    }

    if(sensor_config->bus_handle == nullptr || sensor_config->h_int == GPIO_NUM_NC || sensor_config->rst == GPIO_NUM_NC){

        ESP_LOGE(TAG, "Initialization failed: Invalid input argument. Error Code %u", bno_err_t::INVALID_INPUT);
        return bno_err_t::INVALID_INPUT;
    }

    sc = _shtp.begin(sensor_config->i2c_address, sensor_config->i2c_frequency, sensor_config->bus_handle);

    if(sc != bno_err_t::OK) {
        ESP_LOGE(TAG, "SHTP initialization failed. Error Code %u", sc);
        return sc;
    }
    ESP_LOGI(TAG, "SHTP initialized");

    sc = _SH2.begin(&_shtp, sensor_config->h_int, sensor_config->rst);

    if(sc != bno_err_t::OK) {
        ESP_LOGE(TAG, "SH2 initialization failed. Error Code %u", sc);
        return sc;
    }
    ESP_LOGI(TAG, "SH2 initialized");

    sc = wait_for_init_packages();

    if(sc != bno_err_t::OK) {
        ESP_LOGE(TAG, "Could not receive BNO08x advertisement. Error Code %u", sc);
    }
    ESP_LOGI(TAG, "Initialization complete");

    _initialized = true;

    return bno_err_t::OK;
}

bno_sensor_config_t BNO08x::create_sensor_config(bno_sensor_id_t id, uint16_t frequency_hz) {

    bno_sensor_config_t config = {

        .sensor_id = (uint8_t)id,
        .feature_flags = 0x00,
        .change_sensitivity = 0x00,
        .report_interval = frequency_hz ? 1'000'000UL / frequency_hz : 0,
        .sensor_spesific_config = 0x00,
        .batch_interval = 0x00
    };

    if(id == bno_sensor_id_t::STEP_DETECTOR) 
        config_sensor_change_sensitivity(config, true, false, 1.0f);

    return config;
}

void BNO08x::config_sensor_wakeup(bno_sensor_config_t& config, bool wakeup_enable) {

    namespace mask = bno_constants::feature_flags;

    if(wakeup_enable) 
        config.feature_flags |=  mask::WAKEUP_ENABLED;
    else
        config.feature_flags &= ~mask::WAKEUP_ENABLED;
}

void BNO08x::config_sensor_always_on(bno_sensor_config_t& config, bool always_on_enable) {

    namespace mask = bno_constants::feature_flags;
    
    if(always_on_enable) 
        config.feature_flags |=  mask::ALWAYS_ON_ENABLED;
    else
        config.feature_flags &= ~mask::ALWAYS_ON_ENABLED;
}

void BNO08x::config_sensor_change_sensitivity(bno_sensor_config_t& config, bool change_sensitivity_enable, bool change_sensitivity_relative, float diff_to_event_trigger) {

    namespace mask = bno_constants::feature_flags;

    if(change_sensitivity_enable)
        config.feature_flags |=  mask::CHANGE_SENSITIVITY_ENABLED;
    else
        config.feature_flags &= ~mask::CHANGE_SENSITIVITY_ENABLED;

    if(change_sensitivity_relative)
        config.feature_flags |=  mask::CHANGE_SENSITIVITY_RELATIVE;
    else
        config.feature_flags &= ~mask::CHANGE_SENSITIVITY_RELATIVE;

    config.change_sensitivity = compute_change_sensitivity(config.sensor_id, diff_to_event_trigger);
}

uint16_t BNO08x::compute_change_sensitivity(uint8_t sensor_id, float change_sensitivity) {

    namespace scale = bno_constants::scale_factor;
    namespace sensor = bno_constants::sensor;
    float sf;
    bool compute_uint = false;
    bool sensor_does_not_have_change_sensitivity = false;

    switch (sensor_id)
    {
    case sensor::ACCELEROMETER.id:
    case sensor::LINEAR_ACCELERATION.id:
    case sensor::GRAVITY.id:
        sf = scale::ACCELEROMETER;
        break;
    case sensor::GYROSCOPE.id:
    case sensor::UNCALIBRATED_GYRO.id:
        sf = scale::GYROSCOPE;
        break;
    case sensor::MAGNETIC_FIELD.id:
    case sensor::UNCALIBRATED_MAGNETIC_FIELD.id:
        sf = scale::MAGNETOMETER;
        break;
    case sensor::ROTATION_VECTOR.id:
    case sensor::GAME_ROTATION_VECTOR.id:
    case sensor::GEOMAGNETIC_ROTATION_VECTOR.id:
    case sensor::AR_VR_STABILIZED_ROTATION_VECTOR.id:
    case sensor::AR_VR_STABILIZED_GAME_ROTATION_VECTOR.id:
        compute_uint = true;
        sf = scale::ROTATION_VECTOR_CHANGE_SENSITIVITY;
        break;
    case sensor::RAW_ACCELEROMETER.id:
    case sensor::RAW_GYROSCOPE.id:
    case sensor::RAW_MAGNETOMETER.id:
        sf = 1.0f;
        break;
    case sensor::TEMPERATURE.id:
        sf = scale::TEMPERATURE;
        break;
    case sensor::HUMIDITY.id:
        sf = scale::HUMIDITY;
        break;
    case sensor::PRESSURE.id:
        sf = scale::PRESSURE_CHANGE_SENSITIVITY;
        break;
    case sensor::TAP_DETECTOR.id:
    case sensor::STEP_DETECTOR.id:
    case sensor::STEP_COUNTER.id:
    case sensor::SIGNIFICANT_MOTION_DETECTOR.id:
    case sensor::STABILITY_CLASSIFIER.id:
    case sensor::SHAKE_DETECTOR.id:
    case sensor::FLIP_DETECTOR.id:
    case sensor::PICKUP_DETECTOR.id:
    case sensor::STABILITY_DETECTOR.id:
    case sensor::SLEEP_DETECTOR.id:
    case sensor::TILT_DETECTOR.id:
    case sensor::POCKET_DETECTOR.id:
    case sensor::CIRCLE_DETECTOR.id:
    case sensor::HEART_RATE_MONITOR.id:
        sensor_does_not_have_change_sensitivity = true;
        break;
    case sensor::PERSONAL_ACTIVITY_CLASSIFIER.id:
        sf = 1.0f;
        break;
    default:
        sensor_does_not_have_change_sensitivity = true;
        break;
    }

    if(sensor_does_not_have_change_sensitivity)
        return uint32_t(0);
    if(compute_uint)
        return uint32_t(change_sensitivity / sf);
    else
        return uint32_t(int32_t(change_sensitivity / sf));
}

bno_err_t BNO08x::enable_sensor(const bno_sensor_config_t& config) {

    namespace sensor = bno_constants::sensor;
    bno_err_t sc = bno_err_t::OK;

    if(config.sensor_id == bno_constants::driver_config::BNO_SENSOR_UNDEFINED)
        return bno_err_t::BNO_INVALID_SENSOR_ID;

    uint8_t base_sensor_id = bno_constants::driver_config::BNO_SENSOR_UNDEFINED;

    switch(config.sensor_id)
    {
    case sensor::RAW_ACCELEROMETER.id:
        base_sensor_id = sensor::ACCELEROMETER.id;
        break;
    case sensor::RAW_GYROSCOPE.id:
        base_sensor_id = sensor::GYROSCOPE.id;
        break;
    case sensor::RAW_MAGNETOMETER.id:
        base_sensor_id = sensor::MAGNETIC_FIELD.id;
        break;
    }

    if(base_sensor_id != bno_constants::driver_config::BNO_SENSOR_UNDEFINED) {

        bno_sensor_config_t base_sensor_config;

        base_sensor_config.sensor_id = base_sensor_id;
        base_sensor_config.report_interval = config.report_interval;

        sc = _SH2.set_feature_command(base_sensor_config);
        if(sc != bno_err_t::OK)
            return sc;
    }
    return _SH2.set_feature_command(config);
}

bno_err_t BNO08x::disable_sensor(const bno_sensor_config_t& config) {

    namespace sensor = bno_constants::sensor;
    bno_err_t sc = bno_err_t::OK;
    bno_sensor_config_t disable_config = config;
    disable_config.report_interval = 0;

    uint8_t base_sensor_id = bno_constants::driver_config::BNO_SENSOR_UNDEFINED;

    switch(disable_config.sensor_id)
    {
    case sensor::RAW_ACCELEROMETER.id:
        base_sensor_id = sensor::ACCELEROMETER.id;
        break;
    case sensor::RAW_GYROSCOPE.id:
        base_sensor_id = sensor::GYROSCOPE.id;
        break;
    case sensor::RAW_MAGNETOMETER.id:
        base_sensor_id = sensor::MAGNETIC_FIELD.id;
        break;
    }

    if(base_sensor_id != bno_constants::driver_config::BNO_SENSOR_UNDEFINED) {

        disable_config.sensor_id = base_sensor_id;
        
        sc = _SH2.set_feature_command(disable_config);
        if(sc != bno_err_t::OK)
            return sc;
        
        disable_config.sensor_id = config.sensor_id;
    }

    return _SH2.set_feature_command(disable_config);
}

bno_err_t BNO08x::read_product_id(bno_product_id_t& dest, TickType_t ticks_to_timeout) {

    bno_err_t sc;

    sc = _SH2.send_product_id_request();
    if(sc != bno_err_t::OK)
        return sc;

    return _SH2.get_config_packet(bno_constants::bitmask_config::PRODUCT_ID_RESPONSE, _SH2.storage().config.product_id, dest, ticks_to_timeout);
}

bno_err_t BNO08x::frs_calibrate_env_sensor_temp_hum_pressure(bno_sensor_id_t sensor_id, float target_value, float scale, bool clear_record) {

    bno_err_t sc;
    env_sensor_t env_sensor_data;
    bno_sensor_config_t env_sensor_state;
    bno_sensor_config_t temp_config;
    TickType_t ticks_to_timeout;
    bool sensor_was_off = false;
    
    bno_err_t (BNO08x::*data_getter)(env_sensor_t&, TickType_t);
    float new_offset;
    float sf_offset;

    uint32_t frs_state[2] = {0};
    uint16_t frs_type     = 0;
    size_t words_stored   = 0;

    switch (sensor_id)
    {
    case bno_sensor_id_t::TEMPERATURE:
        data_getter  = &BNO08x::read_temperature;
        sf_offset    = bno_constants::scale_factor::TEMPERATURE;
        frs_type     = bno_constants::frs::config::ENV_SENSOR_TEMPERATURE_CALIBRATION;
        break;
    case bno_sensor_id_t::HUMIDITY:
        data_getter  = &BNO08x::read_humidity;
        sf_offset    = bno_constants::scale_factor::HUMIDITY;
        frs_type     = bno_constants::frs::config::ENV_SENSOR_HUMIDITY_CALIBRATION;
        break;
    case bno_sensor_id_t::PRESSURE:
        data_getter  = &BNO08x::read_pressure;
        sf_offset    = bno_constants::scale_factor::PRESSURE;
        frs_type     = bno_constants::frs::config::ENV_SENSOR_PRESSURE_CALIBRATION;
        break;
    default:
        return bno_err_t::INVALID_INPUT;
    }
    
    if(!clear_record) {

        // 1) Get the current FRS record of the sensor
        sc = _SH2.read_frs_record(frs_type, frs_state, 2, words_stored);
        
        if(sc != bno_err_t::OK)
            return sc;
        if(words_stored < 2)
            return bno_err_t::FRS_NO_DATA_RECEIVED;

        // 2) Check if the user has already enabled the sensor.
        //    If this is the case, keep the user config and do not change anything
        sc = read_sensor_config(sensor_id, env_sensor_state, bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT);
        
        if(sc != bno_err_t::OK)
            return sc;
       
        // If the sensor is not on, we enable it here
        // We also set the timeout for reading a sensor packet to twice the sensor frequency
        if(env_sensor_state.report_interval == 0) {

            temp_config       = create_sensor_config(sensor_id, 10);
            ticks_to_timeout  = pdMS_TO_TICKS((temp_config.report_interval * 2) / 1000);
            sensor_was_off    = true;
            sc                = enable_sensor(temp_config);
            
            if(sc != bno_err_t::OK)
                return sc;
            
        }
        else ticks_to_timeout = pdMS_TO_TICKS((env_sensor_state.report_interval * 2) / 1000);
        
        sc = (this->*data_getter)(env_sensor_data, ticks_to_timeout);

        if(sc != bno_err_t::OK) {

            if(sensor_was_off)
                disable_sensor(temp_config);
            return sc;
        }
            
        if(sensor_was_off) {
            sc = disable_sensor(temp_config);
            if(sc != bno_err_t::OK) 
                return sc;
        }
    
        // Calc the new offset, based on the desired temperature and the current offset
        float current_offset = sf_offset * float(int32_t(frs_state[0]));
        new_offset = target_value - (env_sensor_data.data - current_offset);
    }
    else {
        new_offset = 0.0f;
        scale      = 1.0f;
    }

    frs_state[0] = uint32_t(int32_t(std::round(new_offset / sf_offset)));
    frs_state[1] = uint32_t(int32_t(std::round(scale      / bno_constants::scale_factor::ENV_SENSOR_SCALE_FACTOR)));
    
    return _SH2.write_frs_record(frs_type, frs_state, 2);
}

bno_err_t BNO08x::frs_configure_stability_classifier(float delta_orientation, float stable_threshold, uint8_t delta_acceleration, uint16_t stable_duration) {

    namespace sf = bno_constants::scale_factor;
    uint32_t words[3] = {0};

    words[0] = uint32_t(int32_t(delta_orientation / sf::STABILITY_CLASSIFIER_DELTA_ORIENTATION));
    words[1] = uint32_t(int32_t(stable_threshold / sf::STABILITY_CLASSIFIER_STABLE_THRESHOLD));
    words[2] = ((uint32_t)delta_acceleration) << 16 | (stable_duration);
    
    return _SH2.write_frs_record(bno_constants::frs::config::MOTION_ENGINE_POWER_MANAGEMENT_STABILITY_CLASSIFIER, words, 3);
}   

bno_err_t BNO08x::frs_configure_significant_motion_detector(float acceleration_threshold, uint32_t step_threshold) {

    namespace sf = bno_constants::scale_factor;
    uint32_t words[2] = {0};

    words[0] = uint32_t(int32_t(acceleration_threshold / sf::SIGNIFICANT_MOTION_DETECTOR_ACCEL_THRESHOLD));
    words[1] = step_threshold;

    return _SH2.write_frs_record(bno_constants::frs::config::SIGNIFICANT_MOTION_DETECTOR, words, 2);
}

bno_err_t BNO08x::frs_read_serial_number(uint32_t& dest) {

    bno_err_t sc;
    size_t words_read = 0;

    sc = _SH2.read_frs_record(bno_constants::frs::config::SERIAL_NUMBER, &dest, 1, words_read);

    if(sc != bno_err_t::OK)
        return sc;

    if(words_read == 0)
        return bno_err_t::FRS_NO_DATA_RECEIVED;

    return bno_err_t::OK;
}

bno_err_t BNO08x::wait_for_init_packages() {

    bno_err_t sc;
    bno_device_state_t state;
    namespace mask = bno_constants::bitmask_config;

    // 1) Packet
    sc = _SH2.get_config_packet(mask::ADVERTISEMENT_PACKET, _SH2.storage().metadata.device_state, state, pdMS_TO_TICKS(500));
    if(sc != bno_err_t::OK)
        return sc;

    // 2) Packet
    sc = _SH2.get_config_packet(mask::RESET_RESPONSE_EXECUTABLE, _SH2.storage().metadata.device_state, state, pdMS_TO_TICKS(500));
    if(sc != bno_err_t::OK)
        return sc;

    // 3) Packet
    return _SH2.get_config_packet(mask::COMMAND_INITIALIZED, _SH2.storage().metadata.device_state, state, pdMS_TO_TICKS(500));
}

bno_err_t BNO08x::soft_reset() {

    namespace mask = bno_constants::bitmask_config;
    bno_err_t sc;

    // Make sure we are not reading old unprocessed packages
    _SH2.clear_config_mask(mask::COMMAND_INITIALIZED | mask::RESET_RESPONSE_EXECUTABLE | mask::ADVERTISEMENT_PACKET);

    sc = _SH2.send_executable_command(bno_constants::executable_command::RESET);
    if(sc != bno_err_t::OK)
        return sc;

    return wait_for_init_packages();
}

bno_err_t BNO08x::hard_reset() {

    namespace mask = bno_constants::bitmask_config;
    bno_err_t sc;

    // Make sure we are not reading old unprocessed packages
    _SH2.clear_config_mask(mask::COMMAND_INITIALIZED | mask::RESET_RESPONSE_EXECUTABLE | mask::ADVERTISEMENT_PACKET);

    sc = _SH2.disable_sensor();
    if(sc != bno_err_t::OK)
        return sc;

    vTaskDelay(pdMS_TO_TICKS(10));
    
    sc = _SH2.enable_sensor();

    if(sc != bno_err_t::OK)
        return sc;

    return wait_for_init_packages();
}

bno_err_t BNO08x::read_sensor_config(bno_sensor_id_t sensor_id, bno_sensor_config_t& dest, TickType_t ticks_to_timeout) {

    bno_err_t sc;
    bno_sensor_config_t temp_config_storage;

    _SH2.clear_config_mask(bno_constants::bitmask_config::GET_FEATURE_RESPONSE);

    sc = _SH2.get_feature_request((uint8_t)sensor_id);

    if(sc != bno_err_t::OK)
        return sc;

    while(temp_config_storage.sensor_id != (uint8_t)sensor_id) {

        sc = _SH2.get_config_packet(bno_constants::bitmask_config::GET_FEATURE_RESPONSE, _SH2.storage().config.sensor_config, temp_config_storage, ticks_to_timeout);

        if(sc != bno_err_t::OK)
            return sc;
    }

    dest = temp_config_storage;
    
    return bno_err_t::OK;
}

bno_err_t BNO08x::tare_now(bno_tare_sensor_t sensor, bno_tare_axis_t axis) {

    return _SH2.send_command_tare_now(sensor, axis);
}

bno_err_t BNO08x::clear_tare() {

    return _SH2.send_command_set_reorientation(0.0f ,0.0f ,0.0f, 0.0f);
}

bno_err_t BNO08x::set_reorientation(float w, float x, float y, float z) {

    return _SH2.send_command_set_reorientation(w,x,y,z);
}

bno_err_t BNO08x::read_motion_engine_calibration_config(me_calibration_config_t& dest, TickType_t ticks_to_timeout) {

    namespace mask = bno_constants::bitmask_config;
    bno_err_t sc;

    _SH2.clear_config_mask(mask::COMMAND_ME_CALIBRATION_RESPONSE);

    sc = _SH2.send_command_get_me_calibration();
    if(sc != bno_err_t::OK)
        return sc;

    return _SH2.get_config_packet(mask::COMMAND_ME_CALIBRATION_RESPONSE, _SH2.storage().command.me_calibration.config, dest, ticks_to_timeout);
}

bno_err_t BNO08x::set_motion_engine_calibration_configuration(const me_calibration_config_t config, TickType_t ticks_to_timeout) {

    namespace mask = bno_constants::bitmask_config;
    bno_err_t sc;

    _SH2.clear_config_mask(mask::COMMAND_ME_CALIBRATION_RESPONSE);

    sc = _SH2.send_command_configure_me_calibration(config);
    if(sc != bno_err_t::OK)
        return sc;

    bool success;

    sc = _SH2.get_config_packet(mask::COMMAND_ME_CALIBRATION_RESPONSE, _SH2.storage().command.me_calibration.configuration_successful, success, ticks_to_timeout);
    if(sc != bno_err_t::OK)
        return sc;

    if(success)
        return bno_err_t::OK;
    else 
        return bno_err_t::BNO_ME_CONFIGURATION_ERROR;
}

bno_err_t BNO08x::read_oscillator_type(bno_oscillator_type_t& dest, TickType_t ticks_to_timeout) {

    namespace mask = bno_constants::bitmask_config;
    bno_err_t sc;

    _SH2.clear_config_mask(mask::COMMAND_OSCILLATOR_TYPE_RESPONSE);

    sc = _SH2.send_command_get_oscillator_type();
    if(sc != bno_err_t::OK)
        return sc;

    return _SH2.get_config_packet(mask::COMMAND_OSCILLATOR_TYPE_RESPONSE, _SH2.storage().command.oscillator.type, dest, ticks_to_timeout);
}
