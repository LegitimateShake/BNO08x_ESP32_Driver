#include "bno_parser.h"

bno_err_t bno_parser::begin(bno_data_t* data_storage, data_access_sync* data_exchange) {

    if(_initialized)
        return bno_err_t::OK;

    if(data_storage == nullptr || data_exchange == nullptr)
        return bno_err_t::INVALID_INPUT;

    _data_access = data_exchange;
    _storage     = data_storage;
    _initialized = true;

    return bno_err_t::OK;
}

bno_err_t bno_parser::parse(const shtp_packet_t* rx_packet) {

    if(rx_packet == nullptr)
        return bno_err_t::INVALID_INPUT;

    _rx_packet = rx_packet;

    return parse_channel();
}

bno_err_t bno_parser::parse_channel() {

    namespace channel = bno_constants::channel;

    if(!_initialized)
        return bno_err_t::PARSER_OBJECT_NOT_INITIALIZED;

    bno_err_t sc = bno_err_t::SH2_INVALID_CHANNEL;

    switch (_rx_packet->header.channel)
    {
    case channel::COMMAND:
        sc = parse_advertisement_package();
        break;
    case channel::EXECUTABLE:
        sc = parse_channel_executable();
        break;
    case channel::CONTROL:
        sc = parse_channel_control();
        break;
    case channel::REPORTS:
        sc = parse_channel_reports();
        break;
    case channel::WAKE_REPORTS:
        sc = parse_channel_reports();
        break;
    case channel::GYRO:
        sc = parse_channel_gyro();
        break;
    default:
        sc = bno_err_t::SH2_INVALID_CHANNEL;
    }

    _data_access->unlock();

    if(sc == bno_err_t::OK)
        _data_access->notify_user();

    return sc;
}

bno_err_t bno_parser::parse_channel_executable() {

    if(_rx_packet->size < bno_constants::control_type::packet_size::EXECUTABLE)
        return bno_err_t::SH2_INVALID_REPORT_LENGTH;

    _data_access->sensor_config().set_mask_and_lock(bno_constants::bitmask_config::RESET_RESPONSE_EXECUTABLE);

    if(_rx_packet->data[0] <= bno_constants::executable_command::MAX_VALUE)
        _storage->metadata.device_state = (bno_device_state_t)_rx_packet->data[0]; 
    else 
        _storage->metadata.device_state = bno_device_state_t::RESERVED;

    return bno_err_t::OK;
}

bno_err_t bno_parser::parse_channel_reports() {

    namespace sensor = bno_constants::sensor;
    namespace scale  = bno_constants::scale_factor;
    namespace offset = bno_constants::data_offset::report::metadata;

    uint8_t reportID = _rx_packet->data[offset::REPORT_ID];
    bno_err_t sc = bno_err_t::SH2_INVALID_REPORT_ID;

    switch (reportID)
    {
    case sensor::ACCELEROMETER.id:
        sc = parse_calibrated_accel_gyro_mag(_storage->sensor.acceleration, sensor::ACCELEROMETER.report_length, sensor::ACCELEROMETER.scale_factor);
        break;
    case sensor::GYROSCOPE.id:
        sc = parse_calibrated_accel_gyro_mag(_storage->sensor.gyroscope, sensor::GYROSCOPE.report_length, sensor::GYROSCOPE.scale_factor);
        break;
    case sensor::MAGNETIC_FIELD.id:
        sc = parse_calibrated_accel_gyro_mag(_storage->sensor.magnetic_field, sensor::MAGNETIC_FIELD.report_length, sensor::MAGNETIC_FIELD.scale_factor);
        break;
    case sensor::LINEAR_ACCELERATION.id:
        sc = parse_calibrated_accel_gyro_mag(_storage->sensor.linear_acceleration, sensor::LINEAR_ACCELERATION.report_length, sensor::LINEAR_ACCELERATION.scale_factor);
        break;
    case sensor::ROTATION_VECTOR.id:
        sc = parse_quaternion(_storage->sensor.rotation_vector, sensor::ROTATION_VECTOR.report_length);
        break;
    case sensor::GRAVITY.id:
        sc = parse_calibrated_accel_gyro_mag(_storage->sensor.gravity, sensor::GRAVITY.report_length, sensor::GRAVITY.scale_factor);
        break;
    case sensor::UNCALIBRATED_GYRO.id:
        sc = parse_uncalibrated_gyro_mag(_storage->sensor.uncalibrated_gyroscope, sensor::UNCALIBRATED_GYRO.report_length, sensor::UNCALIBRATED_GYRO.scale_factor);
        break;
    case sensor::GAME_ROTATION_VECTOR.id:
        sc = parse_quaternion(_storage->sensor.game_rotation_vector, sensor::GAME_ROTATION_VECTOR.report_length);
        break;
    case sensor::GEOMAGNETIC_ROTATION_VECTOR.id:
        sc = parse_quaternion(_storage->sensor.geomagnetic_rotation_vector, sensor::GEOMAGNETIC_ROTATION_VECTOR.report_length);
        break;
    case sensor::PRESSURE.id:
        sc = parse_env_sensor(_storage->sensor.pressure, sensor::PRESSURE.report_length, 4, scale::PRESSURE);
        break;
    case sensor::AMBIENT_LIGHT.id:
        break;
    case sensor::HUMIDITY.id:
        sc = parse_env_sensor(_storage->sensor.humidity, sensor::HUMIDITY.report_length, 2, scale::HUMIDITY);
        break;
    case sensor::PROXIMITY.id:
        break;
    case sensor::TEMPERATURE.id:
        sc = parse_env_sensor(_storage->sensor.temperature, sensor::TEMPERATURE.report_length, 2, scale::TEMPERATURE);
        break;
    case sensor::UNCALIBRATED_MAGNETIC_FIELD.id:
        sc = parse_uncalibrated_gyro_mag(_storage->sensor.uncalibrated_magnetic_field, sensor::UNCALIBRATED_MAGNETIC_FIELD.report_length, sensor::UNCALIBRATED_MAGNETIC_FIELD.scale_factor);
        break;
    case sensor::TAP_DETECTOR.id:
        break;
    case sensor::STEP_COUNTER.id:
        break;
    case sensor::SIGNIFICANT_MOTION_DETECTOR.id:
        sc = parse_significant_motion_detector(_storage->sensor.significant_motion_detector, sensor::SIGNIFICANT_MOTION_DETECTOR.report_length);
        break;
    case sensor::STABILITY_CLASSIFIER.id:
        sc = parse_stability_classifier(_storage->sensor.stablitity_classifier, sensor::STABILITY_CLASSIFIER.report_length);
        break;
    case sensor::RAW_ACCELEROMETER.id:
        sc = parse_raw_accel_gyro_mag(_storage->sensor.raw_acceleration, sensor::RAW_ACCELEROMETER.report_length);
        break;
    case sensor::RAW_GYROSCOPE.id:
        sc = parse_raw_accel_gyro_mag(_storage->sensor.raw_gyroscope, sensor::RAW_GYROSCOPE.report_length);
        break;
    case sensor::RAW_MAGNETOMETER.id:
        sc = parse_raw_accel_gyro_mag(_storage->sensor.raw_magnetometer, sensor::RAW_MAGNETOMETER.report_length);
        break;
    case sensor::SHAKE_DETECTOR.id:
        break;
    case sensor::FLIP_DETECTOR.id:
        break;
    case sensor::PICKUP_DETECTOR.id:
        break;
    case sensor::STABILITY_DETECTOR.id:
        sc = parse_stability_detector(_storage->sensor.stability_detector, sensor::STABILITY_DETECTOR.report_length);
        break;
    case sensor::PERSONAL_ACTIVITY_CLASSIFIER.id:
        break;
    case sensor::SLEEP_DETECTOR.id:
        break;
    case sensor::TILT_DETECTOR.id:
        break;
    case sensor::POCKET_DETECTOR.id:
        break;
    case sensor::CIRCLE_DETECTOR.id:
        break;
    case sensor::HEART_RATE_MONITOR.id:
        break;
    case sensor::AR_VR_STABILIZED_ROTATION_VECTOR.id:
        sc = parse_quaternion(_storage->sensor.ar_vr_stabilized_rotation_vector, sensor::AR_VR_STABILIZED_ROTATION_VECTOR.report_length);
        break;
    case sensor::AR_VR_STABILIZED_GAME_ROTATION_VECTOR.id:
        sc = parse_quaternion(_storage->sensor.ar_vr_stabilized_game_rotation_vector, sensor::AR_VR_STABILIZED_GAME_ROTATION_VECTOR.report_length);
        break;
    default:
        sc = bno_err_t::SH2_INVALID_REPORT_ID;
    }
    return sc;
}

bno_err_t bno_parser::parse_channel_gyro() {

    namespace scale  = bno_constants::scale_factor;
    namespace offset = bno_constants::data_offset::report::gyro_integrated_rotation_vector;

    if(_rx_packet->size < bno_constants::sensor::GYRO_INTEGRATED_ROTATION_VECTOR.report_length)
        return bno_err_t::SH2_INVALID_REPORT_LENGTH;

    _data_access->sensor_data().set_bit_and_lock(bno_constants::sensor::GYRO_INTEGRATED_ROTATION_VECTOR.id);

    const uint8_t* data = _rx_packet->data;
    gyro_integrated_rot_vec_t& dest = _storage->sensor.gyro_integrated_rotation_vector;
    
    dest.x = scale::ROTATION_VECTOR * float(read_int16_t(&data[offset::quaternion::X]));
    dest.y = scale::ROTATION_VECTOR * float(read_int16_t(&data[offset::quaternion::Y]));
    dest.z = scale::ROTATION_VECTOR * float(read_int16_t(&data[offset::quaternion::Z]));
    dest.w = scale::ROTATION_VECTOR * float(read_int16_t(&data[offset::quaternion::W]));
    
    dest.vel_x = scale::ANGULAR_VELOCITY * float(read_int16_t(&data[offset::angular_velocity::X]));
    dest.vel_y = scale::ANGULAR_VELOCITY * float(read_int16_t(&data[offset::angular_velocity::Y]));
    dest.vel_z = scale::ANGULAR_VELOCITY * float(read_int16_t(&data[offset::angular_velocity::Z]));
    
    return bno_err_t::OK;
}

void bno_parser::parse_report_metadata(sensor_metadata_t& dest) {

    namespace offset = bno_constants::data_offset::report::metadata;
    const uint8_t* data = _rx_packet->data;

    dest.base_timestamp_reference = read_int32_t(&data[offset::BASE_TIMESTAMP_REFERENCE]);

    dest.sequence_number = data[offset::SEQUENCE_NUMBER];
    dest.accuracy        = (bno_sensor_accuracy_t)(data[offset::ACCURACY_STATUS] & 0x03);
    dest.data_delay      = uint16_t(data[offset::ACCURACY_STATUS] & 0xFC) << 6 | data[offset::REPORT_DELAY];
}

bno_err_t bno_parser::parse_env_sensor(env_sensor_t& dest, uint8_t report_length, uint8_t data_bytes, float scale_factor) {

    namespace offset = bno_constants::data_offset::report;

    if(_rx_packet->size < report_length)
        return bno_err_t::SH2_INVALID_REPORT_LENGTH;
    
    _data_access->sensor_data().set_bit_and_lock(_rx_packet->data[offset::metadata::REPORT_ID]);

    parse_report_metadata(dest.metadata);
    const uint8_t* data = _rx_packet->data;

    if(data_bytes == 2) {
        
        dest.data = scale_factor * float(read_int16_t(&data[offset::generic::SENSOR_DATA_START]));
    }
    else if(data_bytes == 4) {

        dest.data = scale_factor * float(read_int32_t(&data[offset::generic::SENSOR_DATA_START]));
    }
    
    return bno_err_t::OK;
}

bno_err_t bno_parser::parse_quaternion(quaternion_t& dest, uint8_t report_length) {

    namespace sensor = bno_constants::sensor;
    namespace scale  = bno_constants::scale_factor;
    namespace offset = bno_constants::data_offset::report;

    if(_rx_packet->size < report_length)
        return bno_err_t::SH2_INVALID_REPORT_LENGTH;

    _data_access->sensor_data().set_bit_and_lock(_rx_packet->data[offset::metadata::REPORT_ID]);

    parse_report_metadata(dest.metadata);
    const uint8_t* data = _rx_packet->data;

    dest.x = scale::ROTATION_VECTOR * float(read_int16_t(&data[offset::quaternion::X]));
    dest.y = scale::ROTATION_VECTOR * float(read_int16_t(&data[offset::quaternion::Y]));
    dest.z = scale::ROTATION_VECTOR * float(read_int16_t(&data[offset::quaternion::Z]));
    dest.w = scale::ROTATION_VECTOR * float(read_int16_t(&data[offset::quaternion::W]));
    
    if(report_length == sensor::ROTATION_VECTOR.report_length) {
        dest.heading_accuracy_estimate = scale::ROTATION_VECTOR_ACCURACY * float(read_int16_t(&data[offset::quaternion::ACCURACY]));
    }
    
    return bno_err_t::OK;
}

bno_err_t bno_parser::parse_calibrated_accel_gyro_mag(accel_gyro_mag_t& dest, uint8_t report_length, float scale_factor) {

    namespace offset = bno_constants::data_offset::report;

    if(_rx_packet->size < report_length)
        return bno_err_t::SH2_INVALID_REPORT_LENGTH;

    _data_access->sensor_data().set_bit_and_lock(_rx_packet->data[offset::metadata::REPORT_ID]);

    parse_report_metadata(dest.metadata);
    const uint8_t* data = _rx_packet->data;

    dest.x = scale_factor * float(read_int16_t(&data[offset::accel_gyro_mag::X]));
    dest.y = scale_factor * float(read_int16_t(&data[offset::accel_gyro_mag::Y]));
    dest.z = scale_factor * float(read_int16_t(&data[offset::accel_gyro_mag::Z]));
    
    return bno_err_t::OK;
}

bno_err_t bno_parser::parse_uncalibrated_gyro_mag(uncalibrated_gyro_mag_t& dest, uint8_t report_length, float scale_factor) {

    namespace offset = bno_constants::data_offset::report;

    if(_rx_packet->size < report_length)
        return bno_err_t::SH2_INVALID_REPORT_LENGTH;

    _data_access->sensor_data().set_bit_and_lock(_rx_packet->data[offset::metadata::REPORT_ID]);

    parse_report_metadata(dest.metadata);
    const uint8_t* data = _rx_packet->data;

    dest.x = scale_factor * float(read_int16_t(&data[offset::accel_gyro_mag::X]));
    dest.y = scale_factor * float(read_int16_t(&data[offset::accel_gyro_mag::Y]));
    dest.z = scale_factor * float(read_int16_t(&data[offset::accel_gyro_mag::Z]));

    dest.bias_x = scale_factor * float(read_int16_t(&data[offset::accel_gyro_mag::BIAS_X]));
    dest.bias_y = scale_factor * float(read_int16_t(&data[offset::accel_gyro_mag::BIAS_Y]));
    dest.bias_z = scale_factor * float(read_int16_t(&data[offset::accel_gyro_mag::BIAS_Z]));

    return bno_err_t::OK;
}

bno_err_t bno_parser::parse_raw_accel_gyro_mag(raw_accel_gyro_mag_t& dest, uint8_t report_length) {

    namespace offset = bno_constants::data_offset::report;

    if(_rx_packet->size < report_length)
        return bno_err_t::SH2_INVALID_REPORT_LENGTH;

    _data_access->sensor_data().set_bit_and_lock(_rx_packet->data[offset::metadata::REPORT_ID]);

    parse_report_metadata(dest.metadata);
    const uint8_t* data = _rx_packet->data;

    dest.x = read_int16_t(&data[offset::accel_gyro_mag::X]);
    dest.y = read_int16_t(&data[offset::accel_gyro_mag::Y]);
    dest.z = read_int16_t(&data[offset::accel_gyro_mag::Z]);

    if(_rx_packet->data[offset::metadata::REPORT_ID] == bno_constants::sensor::RAW_GYROSCOPE.id) {

        dest.temperature = read_int16_t(&data[offset::accel_gyro_mag::RAW_GYRO_TEMP]);
    }

    dest.timestamp = read_uint32_t(&data[offset::accel_gyro_mag::RAW_IMESTAMP]);

    return bno_err_t::OK;
}

bno_err_t bno_parser::parse_stability_classifier(stability_classifier_t& dest, uint8_t report_length) {

    namespace offset = bno_constants::data_offset::report;

    if(_rx_packet->size < report_length)
        return bno_err_t::SH2_INVALID_REPORT_LENGTH;

    _data_access->sensor_data().set_bit_and_lock(_rx_packet->data[offset::metadata::REPORT_ID]);

    parse_report_metadata(dest.metadata);
    const uint8_t* data = _rx_packet->data;

    if(data[offset::generic::SENSOR_DATA_START] <= bno_constants::data::sensor::stability_classifier::MAX_VALUE)
        dest.state = (bno_stablitity_state_t)data[offset::generic::SENSOR_DATA_START];
    else
        dest.state = bno_stablitity_state_t::UNKNOWN;

    return bno_err_t::OK;
}

bno_err_t bno_parser::parse_stability_detector(stability_detector_t& dest, uint8_t report_length) {

    namespace offset = bno_constants::data_offset::report;

    if(_rx_packet->size < report_length)
        return bno_err_t::SH2_INVALID_REPORT_LENGTH;

    _data_access->sensor_data().set_bit_and_lock(_rx_packet->data[offset::metadata::REPORT_ID]);

    parse_report_metadata(dest.metadata);
    
    uint8_t bitmap = _rx_packet->data[offset::generic::SENSOR_DATA_START];

    switch (bitmap & 0b11) {
    case 0b01:
        dest.state = bno_stablitity_state_t::STABLE;
        break;
    case 0b10:
        dest.state = bno_stablitity_state_t::IN_MOTION;
        break;
    }
    return bno_err_t::OK;
}

bno_err_t bno_parser::parse_significant_motion_detector(significant_motion_detector_t& dest, uint8_t report_length) {

    namespace offset = bno_constants::data_offset::report;

    if(_rx_packet->size < report_length)
        return bno_err_t::SH2_INVALID_REPORT_LENGTH;

    _data_access->sensor_data().set_bit_and_lock(_rx_packet->data[offset::metadata::REPORT_ID]);

    dest.significant_motion = bool(read_uint16_t(&_rx_packet->data[offset::generic::SENSOR_DATA_START]));

    return bno_err_t::OK;
}

bno_err_t bno_parser::parse_channel_control() {

    namespace control = bno_constants::control_type::id;
    namespace size    = bno_constants::control_type::packet_size;
    bno_err_t sc = bno_err_t::SH2_INVALID_CONTROL_ID;

    uint8_t control_type = _rx_packet->data[0];
    
    switch (control_type)
    {
    case control::COMMAND_RESPONSE:
        sc = parse_command_response();
        break;
    case control::FRS_WRITE_RESPONSE:
        sc = parse_frs_write_response(_storage->frs.write_response, size::FRS_WRITE_RESPONSE);
        break;
    case control::FRS_READ_RESPONSE:
        sc = parse_frs_read_response(_storage->frs.read_response, size::FRS_READ_RESPONSE);
        break;
    case control::GET_FEATURE_RESPONSE:
        sc = parse_get_feature_response(_storage->config.sensor_config, size::GET_FEATURE_RESPONSE);
        break;
    case control::PRODUCT_ID_RESPONSE:
        sc = parse_product_id_response(_storage->config.product_id, size::PRODUCT_ID_RESPONSE);
        break;

    default:
        sc = bno_err_t::SH2_INVALID_CONTROL_ID;
    }
    return sc;
}

bno_err_t bno_parser::parse_product_id_response(bno_product_id_t&dest, uint8_t report_length) {

    namespace offset = bno_constants::data_offset::config::product_id_response;

    // For some reason another 52 byte packet is received after the original 20 byte packet
    // No clue what this second packet contains. It seems like sw info for other part numbers ?
    // Anyway, for now we discard the second packet and just process the first packet 
    if(_rx_packet->size != report_length)
        return bno_err_t::SH2_INVALID_REPORT_LENGTH;
    
    _data_access->sensor_config().set_mask_and_lock(bno_constants::bitmask_config::PRODUCT_ID_RESPONSE);
    
    const uint8_t* data = _rx_packet->data;

    dest.reset_cause            = data[offset::RESET_CAUSE];
    dest.software_version_major = data[offset::SW_VERSION_MAJOR];
    dest.software_version_minor = data[offset::SW_VERSION_MINOR];
    dest.software_part_number   = read_uint32_t(&data[offset::SW_PART_NUMBER]);
    dest.software_build_number  = read_uint32_t(&data[offset::SW_BUILD_NUMBER]);
    dest.software_patch_number  = read_uint16_t(&data[offset::SW_VERSION_PATCH]);

    return bno_err_t::OK;
}


bno_err_t bno_parser::parse_get_feature_response(bno_sensor_config_t& dest, uint8_t report_length) {

    namespace offset = bno_constants::data_offset::config::feature_response;

    if(_rx_packet->size < report_length)
        return bno_err_t::SH2_INVALID_REPORT_LENGTH;

    _data_access->sensor_config().set_mask_and_lock(bno_constants::bitmask_config::GET_FEATURE_RESPONSE);
    
    const uint8_t* data = _rx_packet->data;

    dest.sensor_id              = data[offset::REPORT_ID];
    dest.feature_flags          = data[offset::FEATURE_FLAGS];
    dest.change_sensitivity     = read_uint16_t(&data[offset::CHANGE_SENSITIVITY]);
    dest.report_interval        = read_uint32_t(&data[offset::REPORT_INTERVAL]);
    dest.batch_interval         = read_uint32_t(&data[offset::BATCH_INTERVAL]);
    dest.sensor_spesific_config = read_uint32_t(&data[offset::SENSOR_SPESIFIC_CONFIGURATION]);

    return bno_err_t::OK;
}

bno_err_t bno_parser::parse_frs_write_response(bno_frs_write_response_t& dest, uint8_t report_length) {

    namespace offset = bno_constants::data_offset::config::frs_write;
    namespace frs_status = bno_constants::frs::write_status;

    if(_rx_packet->size < report_length)
        return bno_err_t::SH2_INVALID_REPORT_LENGTH;

    const uint8_t* data = _rx_packet->data;

    _data_access->sensor_config().set_mask_and_lock(bno_constants::bitmask_config::FRS_WRITE_RESPONSE);

    uint8_t status_code = data[offset::STATUS_CODE];

    if(status_code == frs_status::WRITE_COMPLETED && dest.status_code == frs_status::RECORD_VALID)
        dest.status_code = frs_status::RECORD_VALID_AND_WRITE_COMPLETED;
    else
        dest.status_code = status_code;

    dest.word_offset = read_uint16_t(&data[offset::WORD_OFFSET]);

    return bno_err_t::OK;
}

bno_err_t bno_parser::parse_frs_read_response(bno_frs_read_response_t& dest, uint8_t report_length) {

    namespace offset = bno_constants::data_offset::config::frs_read;
    namespace status = bno_constants::frs::read_status;

    if(_rx_packet->size < report_length)
        return bno_err_t::SH2_INVALID_REPORT_LENGTH;
    
    const uint8_t* data = _rx_packet->data;

    uint8_t data_length_and_status = data[offset::DATA_LENGTH_AND_STATUS];
    uint8_t status_code  = data_length_and_status & 0x0F;
    uint8_t data_length  = data_length_and_status >> 4;
    uint16_t word_offset = read_uint16_t(&data[offset::WORD_OFFSET]);
    uint16_t frs_type    = read_uint16_t(&data[offset::FRS_TYPE]);
    
    if(status_code != status::NO_ERROR)
        _data_access->sensor_config().set_mask_and_lock(bno_constants::bitmask_config::FRS_READ_COMPLETE);
    else
        _data_access->lock();
    
    // Reset if a new record type arrives or the last was completed
    if(dest.transfer_complete || dest.frs_type != frs_type) {
        dest.frs_type = frs_type;
        dest.buffer_overflow = false;
        dest.transfer_complete = false;
        dest.words_stored = 0;
    }

    if(data_length >= 1) {

        size_t index = word_offset;
        if(index < bno_constants::driver_config::FRS_READ_BUFFER_SIZE) {
            dest.words[index] = read_uint32_t(&data[offset::DATA_0]);
            dest.words_stored++;
        }
        else dest.buffer_overflow = true;
    }
    if(data_length == 2) {

        size_t index = word_offset + 1;
        if(index < bno_constants::driver_config::FRS_READ_BUFFER_SIZE) {
            dest.words[index] = read_uint32_t(&data[offset::DATA_1]);
            dest.words_stored++;
        }
        else dest.buffer_overflow = true;
    }
    dest.status_code = status_code;
    
    if(status_code != status::NO_ERROR) {
        dest.transfer_complete = true;
        return bno_err_t::OK;
    }
    else return bno_err_t::FRS_TRANSFER_ONGOING;
}

bno_err_t bno_parser::parse_command_response() {

    namespace command = bno_constants::command;
    namespace scale  = bno_constants::scale_factor;
    namespace offset = bno_constants::data_offset::config::command::metadata;

    bno_err_t sc = bno_err_t::SH2_INVALID_COMMAND_ID;
    uint8_t commandID = _rx_packet->data[offset::R_COMMAND_ID];
    //MSB indicates if the report command is a response to a request or not (not important)
    commandID &= 0x7F;

    switch (commandID)
    {
    case command::INITIALIZE.id:
        sc = parse_command_initialize(_storage->command.initialized, command::INITIALIZE.report_length);
        break;
    case command::TARE.id:
        // This one does not send a response
        break;
    case command::TURNTABLE_CALIBRATION.id:
        // This one seems to not be supported by the bno08x
        break;

    default:
        sc = bno_err_t::SH2_INVALID_COMMAND_ID;
    }

    return sc;
}   

void bno_parser::parse_command_metadata(command_metadata_t& dest) {

    namespace offset = bno_constants::data_offset::config::command::metadata;
    _storage->command.sequence_number = _rx_packet->data[offset::R_SEQUENCE_NUMBER];

    dest.command_sequence_number  = _rx_packet->data[offset::R_COMMAND_SEQUENCE_NUMBER];
    dest.response_sequence_number = _rx_packet->data[offset::R_RESPONSE_SEQUENCE_NUMBER];
}

bno_err_t bno_parser::parse_command_initialize(command_initialized_t& dest, uint8_t report_length) {
    
    namespace offset = bno_constants::data_offset::config::command::initialized;

    if(_rx_packet->size < report_length)
        return bno_err_t::SH2_INVALID_REPORT_LENGTH;

    _data_access->sensor_config().set_mask_and_lock(bno_constants::bitmask_config::COMMAND_INITIALIZED);
    parse_command_metadata(dest.metadata);
    
    dest.status = (bno_intialization_state_t)_rx_packet->data[offset::R_STATUS];

    return bno_err_t::OK;
}

bno_err_t bno_parser::parse_advertisement_package() {
    
    namespace ad = bno_constants::advertisement_packet;

    _data_access->sensor_config().set_mask_and_lock(bno_constants::bitmask_config::ADVERTISEMENT_PACKET);

    uint8_t  tag;
    uint8_t  length = 0;
    size_t   index  = 1;
    int32_t  active_guid = -1;
    uint16_t bytes_in_buffer = _rx_packet->size - bno_constants::shtp::SHTP_HEADER_SIZE;

    while(index < bytes_in_buffer) {

        index += length;

        // Full message read
        if(index >= bytes_in_buffer)
            break;

        // Check if there are at least two more bytes in the buffer before reading them
        if(index + 2 > bytes_in_buffer)
            return bno_err_t::SH2_INVALID_REPORT_LENGTH;

        tag    = _rx_packet->data[index++];
        length = _rx_packet->data[index++];

        // Check if all payload bytes are in the buffer          
        if(index + length > bytes_in_buffer)                 
            return bno_err_t::SH2_INVALID_REPORT_LENGTH;

        // Get the active GUID before parsing any values
        if(tag == ad::TAG_GUID && length == ad::TAG_GUID_LENGTH) {

            active_guid = _rx_packet->data[index];
            continue;
        }
        else if(active_guid == -1) 
            continue;

        // Parses the SHTP Version
        if(active_guid == ad::GUID_SHTP && tag == ad::TAG_VERSION && length <= ad::CHAR_INPUT_BUFFER_SIZE) {

            std::memcpy(_storage->metadata.shtp_version, &_rx_packet->data[index], length);
            continue;
        }

        // Parses the Sensorhub Version
        if(active_guid == ad::GUID_SENSORHUB && tag == ad::TAG_VERSION && length <= ad::CHAR_INPUT_BUFFER_SIZE) {

            std::memcpy(_storage->metadata.sensorhub_version, &_rx_packet->data[index], length);
            continue;
        }

        // Parses the max amount of bytes that can be written in one transfer (header + cargo)
        if(tag == ad::TAG_MAX_CARGO_PLUS_HEADER_WRITE && length == ad::TAG_MAX_CARGO_PLUS_HEADER_WRITE_SIZE) {

            uint8_t lsb = _rx_packet->data[index];
            uint8_t msb = _rx_packet->data[index+1];
            _storage->metadata.max_transfer_write = ((uint16_t)msb << 8) | lsb; 
            continue;
        }

        // Parses the max amount of bytes that can be read in one transfer (header+cargo)
        if(tag == ad::TAG_MAX_CARGO_PLUS_HEADER_READ && length == ad::TAG_MAX_CARGO_PLUS_HEADER_READ_SIZE) {

            uint8_t lsb = _rx_packet->data[index];
            uint8_t msb = _rx_packet->data[index+1];
            _storage->metadata.max_transfer_read = ((uint16_t)msb << 8) | lsb;
            continue;
        }
    }

    ESP_LOGI(TAG, 
             "SHTP version: %s, SensorHub version: %s", 
             _storage->metadata.shtp_version, 
             _storage->metadata.sensorhub_version);

    return bno_err_t::OK;
}