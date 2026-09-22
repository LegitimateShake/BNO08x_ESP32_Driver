#pragma once

#include <cstdint>
#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include <shtp.h>
#include <sh2.h>
#include <bno_parser.h>
#include <bno_types.h>
#include <bno_constants.h>

class BNO08x {

public:

    shtp _shtp;
    SH2  _SH2;

    static constexpr char TAG[] = "BNO08x";
    bool _initialized = false;

    bno_err_t frs_calibrate_env_sensor_temp_hum_pressure(bno_sensor_id_t sensor_id, float target_value, float scale, bool clear_record);

    bno_err_t wait_for_init_packages();

    uint16_t compute_change_sensitivity(uint8_t sensor_id, float change_sensitivity);

public:


/**
 * @brief Initializes the sensor. Must be called before any other communication is possible
 * @param sensor_config pointer to the sensor configuration. Must be fully filled
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t begin(bno_config_t* sensor_config);


/**
 * @brief Writes calibration values into the flash of the BNO086
 * @param target_temperature The current temperature the sensor should be returning. Units are degrees celsius
 * @param scale_sensor_output The output of the sensor is multiplied by this factor
 * @param clear_record Set to true to reset the frs record
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t frs_calibrate_temperature(float target_temperature, float scale_sensor_output = 1.0f, bool clear_record = false) 
{
    return frs_calibrate_env_sensor_temp_hum_pressure(bno_sensor_id_t::TEMPERATURE, target_temperature, scale_sensor_output, clear_record);
}


/**
 * @brief Writes calibration values into the flash of the BNO086
 * @param target_humidity The current humidity the sensor should be returning. Units are %
 * @param scale_sensor_output The output of the sensor is multiplied by this factor
 * @param clear_record Set to true to reset the frs record
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t frs_calibrate_humidity(float target_humidity, float scale_sensor_output = 1.0f, bool clear_record = false) 
{
    return frs_calibrate_env_sensor_temp_hum_pressure(bno_sensor_id_t::HUMIDITY, target_humidity, scale_sensor_output, clear_record);
}


/**
 * @brief Writes calibration values into the flash of the BNO086
 * @param target_pressure The current pressure the sensor should be returning. Units are hectopascal
 * @param scale_sensor_output The output of the sensor is multiplied by this factor
 * @param clear_record Set to true to reset the frs record
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t frs_calibrate_pressure(float target_pressure, float scale_sensor_output = 1.0f, bool clear_record = false) 
{
    return frs_calibrate_env_sensor_temp_hum_pressure(bno_sensor_id_t::PRESSURE, target_pressure, scale_sensor_output, clear_record);
}


/**
 * @brief Configures the stability classifier
 * @param delta_orientation Once a device has been determined to be not moving based on the generation of ON_TABLE or IS_STABLE, delta 
 *                          orientation is the amount of change in device orientation required to recognize that the device is in motion. 
 *                          The units are radians.
 * @param stable_threshold The gyro output must be below this limit for the stable duration in order for an IS_STABLE notification to be generated. 
 *                         The units are radians per second.
 * @param delta_acceleration When using wake-on-motion, this is the amount of acceleration required for the accelerometer to determine that motion has occurred. 
 *                           The units are mg.
 * @param stable_duration The amount of time in seconds that motion must be below the stable threshold before an IS_STABLE notification is generated.
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t frs_configure_stability_classifier(float delta_orientation = 0.2f, float stable_threshold = 1.0f, uint8_t delta_acceleration = 0, uint16_t stable_duration = 3);


/**
 * @brief Configures the significant motion detector
 * @param acceleration_threshold The acceleration threshold to trigger significant motion. Units are m/s^2
 * @param step_threshold The number of steps required to trigger significant motion
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t frs_configure_significant_motion_detector(float acceleration_threshold = 10.0, uint32_t step_threshold = 5);


/**
 * @brief Write a Serial Number to the flash of the BNO08x
 * @param serial_number The serial number that should be stored
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t frs_write_serial_number(uint32_t serial_number)
{
    return _SH2.write_frs_record(bno_constants::frs::config::SERIAL_NUMBER, &serial_number, 1);
}


/**
 * @brief Read the Serial Number of the BNO08x. This is a 32-bit number, used to identify an individual device
 * @param dest Reference to where the Serial Number should be stored
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t frs_read_serial_number(uint32_t& dest);


/**
 * @brief Write data into the user record of the BNO08x. The record is stored in flash and can contain up to 64 bytes
 * @param src The data that should be stored
 * @param word_amount The amount of 32-bit words that should be stored. 
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t frs_write_user_record(const uint32_t *src, uint16_t word_amount) 
{
    if(word_amount > 64) 
        word_amount = 64;

    return _SH2.write_frs_record(bno_constants::frs::config::USER_RECORD, src, word_amount);
}


/**
 * @brief Read the user record from the flash of the BNO08x
 * @param dest Destination the record should be copied to
 * @param buffer_size The size of the destination array
 * @param words_stored The amount of words that were stored in the destination array
 * @return bno_err_t status code. `bno_err_t::OK` on success 
 */
inline bno_err_t frs_read_user_record(uint32_t *dest, size_t buffer_size, size_t &words_stored)
{
    return _SH2.read_frs_record(bno_constants::frs::config::USER_RECORD, dest, buffer_size, words_stored);
}


/**
 * @brief Send a packet to the BNO08x, telling it to reset
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t soft_reset();


/**
 * @brief Forces a reset through the RST pin of the BNO08x
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t hard_reset();


/**
 * @brief Puts the BNO08x into sleep mode 
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t enter_sleep() 
{
    return _SH2.send_executable_command(bno_constants::executable_command::SLEEP);
}


/**
 * @brief Pulls the BNO08x out of sleep
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t exit_sleep()
{
    return _SH2.send_executable_command(bno_constants::executable_command::ON);
}


/**
 * @brief Configures which parts of the motion engine calibration system should be turned on/off
 * @param config The configuration that should be sent to the sensor. 
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t set_motion_engine_calibration_configuration(const me_calibration_config_t config, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT);


/**
 * @brief Create a configuration struct for a sensor
 * @param id The id of the sensor
 * @param frequency_hz The output frequency of the sensor
 * @returns The configurated struct
 */
bno_sensor_config_t create_sensor_config(bno_sensor_id_t id, uint16_t frequency_hz);


/**
 * @brief Configurate the sensor ID 
 * @param config The configuration struct that should be adjusted
 * @param id The id of the sensor
*/
inline void config_sensor_id(bno_sensor_config_t& config, bno_sensor_id_t id) 
{
    config.sensor_id = (uint8_t)id;
}


/**
 * @brief Configurate the output frequency
 * @param config The configuration struct that should be adjusted
 * @param frequency_hz The desired output frequency of the sensor in Hz
 */
inline void config_sensor_frequency(bno_sensor_config_t& config, uint16_t frequency_hz) 
{
    config.report_interval = frequency_hz ? 1000000UL / frequency_hz : 0;;
}


/**
 * @brief Configure the batch interval of the sensor
 * @param config The configuration struct that should be adjusted
 * @param batch_interval_us 32-bit unsigned integer controlling the maximum delay (in microseconds) between the time that a sensor is sampled
 *                          and the time that its data can be reported. The value 0 is reserved for “do not delay” and the value 0xFFFFFFFF is
 *                          reserved for “never trigger delivery on the basis of elapsed time”
 */
inline void config_sensor_batch_interval(bno_sensor_config_t& config, uint32_t batch_interval_us) 
{
    config.batch_interval = batch_interval_us;
}


/**
 * @brief Configure if this sensor of the BNO086 should be able to wake up the application processor from sleep when new data is available. 
 * @param config The configuration struct that should be adjusted
 * @param wakeup_enable Set to true to enable that when the sensor has a report to send, it can send this report to the host on the wake channel
*/
void config_sensor_wakeup(bno_sensor_config_t& config, bool wakeup_enable);


/**
 * @brief Configure if the sensor should remain on, even if the BNO08x is set into sleep mode. This only has an effect for the step counter.
 * @param config The configuration struct that should be adjusted
 * @param always_on_enable Set to true if the sensor should remain on, even in sleep mode.
 */
void config_sensor_always_on(bno_sensor_config_t& config, bool always_on_enable);


/**
 * @brief Configure if a sensor should only send new data when the measurement changes by a relative or absolute amount.
 * @brief Only works for Significant Motion Detector and Environment Sensors
 * @param config The configuration struct that should be adjusted
 * @param change_sensitivity_enable Enable or disable the feature
 * @param change_sensitivity_relative Set to false to use an absolute value as a refererence and to true to use the last measurement as a reference
 * @param diff_to_event_trigger If change_sensitivity_relative = false: Absolute value the sensor must exceed
 *                              If change_sensitivity_relative = true:  Difference to the last sesor report the sensor must exceed             
 */
void config_sensor_change_sensitivity(bno_sensor_config_t& config, bool change_sensitivity_enable, bool change_sensitivity_relative, float diff_to_event_trigger);


/**
 * @brief enables the given sensor based on the given configuration
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t enable_sensor(const bno_sensor_config_t& config); 


/**
 * @brief disables output of the given sensor
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t disable_sensor(const bno_sensor_config_t& config); 


/**
 * @brief Tare the quaternion orientation of the sensor.
 * @brief The orientation in which the sensor is tared will be set to `q = (1, 0, 0, 0)`
 * @param sensor The quaternion output that should be tared
 * @param axis Select if only the z axis or all three axis should be tared
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t tare_now(bno_tare_sensor_t sensor, bno_tare_axis_t axis);


/**
 * @brief Clears any performed tare that was not written into the frs of the sensor
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t clear_tare();


/**
 * @brief Configure a reorientation quaternion that will be applied to each quaternion output
 * @param w quaternion real component
 * @param x quaternion x component
 * @param y quaternion y component
 * @param z quaternion z component
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t set_reorientation(float w, float x, float y, float z);


/**
 * @brief Read rotation vector data from the sensor
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_rotation_vector(quaternion_t&dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT)
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::ROTATION_VECTOR.id, _SH2.storage().sensor.rotation_vector, dest, ticks_to_timeout);
}


/**
 * @brief Read game rotation vector data from the sensor
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_game_rotation_vector(quaternion_t&dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT)
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::GAME_ROTATION_VECTOR.id, _SH2.storage().sensor.game_rotation_vector, dest, ticks_to_timeout);
}


/**
 * @brief Read geomagnetic rotation vector data from the sensor
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_geomagnetic_rotation_vector(quaternion_t&dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT)
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::GEOMAGNETIC_ROTATION_VECTOR.id, _SH2.storage().sensor.geomagnetic_rotation_vector, dest, ticks_to_timeout);
}


/**
 * @brief Read ar vr stabilized rotation vector data from the sensor
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_ar_vr_stabilized_rotation_vector(quaternion_t&dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT)
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::AR_VR_STABILIZED_ROTATION_VECTOR.id, _SH2.storage().sensor.ar_vr_stabilized_rotation_vector, dest, ticks_to_timeout);
}


/**
 * @brief Read ar vr stabilized game rotation vector data from the sensor
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_ar_vr_stabilized_game_rotation_vector(quaternion_t&dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT)
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::AR_VR_STABILIZED_GAME_ROTATION_VECTOR.id, _SH2.storage().sensor.ar_vr_stabilized_game_rotation_vector, dest, ticks_to_timeout);
}


/**
 * @brief Read acceleration data from the sensor
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_acceleration(accel_gyro_mag_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT) 
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::ACCELEROMETER.id, _SH2.storage().sensor.acceleration, dest, ticks_to_timeout);
}


/**
 * @brief Read gravitational acceleration data from the sensor
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_gravity(accel_gyro_mag_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT) 
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::GRAVITY.id, _SH2.storage().sensor.gravity, dest, ticks_to_timeout);
}


/**
 * @brief Read acceleration data without gravitational acceleration from the sensor
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_linear_acceleration(accel_gyro_mag_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT) 
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::LINEAR_ACCELERATION.id, _SH2.storage().sensor.linear_acceleration, dest, ticks_to_timeout);
}


/**
 * @brief Read angular velocity data from the sensor
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_gyroscope(accel_gyro_mag_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT) 
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::GYROSCOPE.id, _SH2.storage().sensor.gyroscope, dest, ticks_to_timeout);
}


/**
 * @brief Read magnetic field data from the sensor
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_magnetic_field(accel_gyro_mag_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT) 
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::MAGNETIC_FIELD.id, _SH2.storage().sensor.magnetic_field, dest, ticks_to_timeout);
}


/**
 * @brief Read raw acceleration data from the sensor. These are unitless ADCs
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_raw_acceleration(raw_accel_gyro_mag_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT)
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::RAW_ACCELEROMETER.id, _SH2.storage().sensor.raw_acceleration, dest, ticks_to_timeout);
}


/**
 * @brief Read raw angular velocity data from the sensor. These are unitless ADCs
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_raw_gyroscope(raw_accel_gyro_mag_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT)
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::RAW_GYROSCOPE.id, _SH2.storage().sensor.raw_gyroscope, dest, ticks_to_timeout);
}


/**
 * @brief Read raw magnetic field data from the sensor. These are unitless ADCs
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_raw_magnetic_field(raw_accel_gyro_mag_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT)
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::RAW_MAGNETOMETER.id, _SH2.storage().sensor.raw_magnetometer, dest, ticks_to_timeout);
}


/**
 * @brief Read quaternion and angular velocity data from the sensor.
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_gyro_integrated_rotation_vector(gyro_integrated_rot_vec_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT) 
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::GYRO_INTEGRATED_ROTATION_VECTOR.id, _SH2.storage().sensor.gyro_integrated_rotation_vector, dest, ticks_to_timeout);
}


/**
 * @brief Read temperature data from the sensor. Units are degrees celsius
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_temperature(env_sensor_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT) 
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::TEMPERATURE.id, _SH2.storage().sensor.temperature, dest, ticks_to_timeout);
}


/**
 * @brief Read temperature data from the sensor. Units are hectopascals
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_pressure(env_sensor_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT) 
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::PRESSURE.id, _SH2.storage().sensor.pressure, dest, ticks_to_timeout);
}


/**
 * @brief Read temperature data from the sensor. Units are percent
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_humidity(env_sensor_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT) 
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::HUMIDITY.id, _SH2.storage().sensor.humidity, dest, ticks_to_timeout);
}


/**
 * @brief Read the stability state of the sensor
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_stability_classifier(stability_classifier_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT)
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::STABILITY_CLASSIFIER.id, _SH2.storage().sensor.stablitity_classifier, dest, ticks_to_timeout);
}


/**
 * @brief Read the stability state of the sensor
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_stability_detector(stability_detector_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT)
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::STABILITY_DETECTOR.id, _SH2.storage().sensor.stability_detector, dest, ticks_to_timeout);
}


/**
 * @brief Get notified when the sensor detects significant motion. This sensor returns a single packet and shuts off afterwards
 * @brief By default, this function blocks until the sensor notification arrives
 * @param dest Reference to a struct that should be filled with the sensor data
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
inline bno_err_t read_significant_motion_detector(significant_motion_detector_t&dest, TickType_t ticks_to_timeout = portMAX_DELAY)
{
    return _SH2.get_sensor_data_packet(bno_constants::sensor::SIGNIFICANT_MOTION_DETECTOR.id, _SH2.storage().sensor.significant_motion_detector, dest, ticks_to_timeout);
}


/**
 * @brief Read the current configuration of any sensor
 * @param sensor_id Id of the sensor for which the configuration should be returned
 * @param dest Reference to a struct that should be filled with the sensor configuration
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t read_sensor_config(bno_sensor_id_t sensor_id, bno_sensor_config_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT);


/**
 * @brief Read the product id of the sensor
 * @param dest Reference to a struct that should be filled with the product id
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t read_product_id(bno_product_id_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT);


/**
 * @brief Read the current motion engine calibration configuration of the sensor
 * @param dest Reference to a struct that should be filled with the motion engine calibration configuration
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted
 * @return bno_err_t status code. `bno_err_t::OK` on success
 */
bno_err_t read_motion_engine_calibration_config(me_calibration_config_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT);


/**
 * @brief Get the type of oscillator the sensor is using
 * @param dest Reference to the variable that the type of oscillator should be written to 
 * @param ticks_to_timeout Amount of ticks the function waits for a packet before the read is aborted 
 */
bno_err_t read_oscillator_type(bno_oscillator_type_t& dest, TickType_t ticks_to_timeout = bno_constants::driver_config::BNO_TICKS_TO_TIMEOUT);

};
        
