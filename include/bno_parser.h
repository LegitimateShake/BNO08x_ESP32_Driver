#pragma once
#include <cstring>
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include <bno_types.h>
#include <bno_constants.h>
#include <shtp.h>

class bno_parser {

    public:

        bno_err_t begin(bno_data_t* data_storage, data_access_sync* data_exchange);

        bool is_initialized()
        {
            return _initialized;
        }

        bno_err_t parse(const shtp_packet_t* rx_packet);

    private:

        bno_data_t* _storage                = nullptr;
        data_access_sync* _data_access      = nullptr;
        const shtp_packet_t* _rx_packet     = nullptr;

        static constexpr char TAG[] = "SH2-Parser";

        bool _initialized = false;

        static int16_t  read_int16_t(const uint8_t* data)
        {
            return static_cast<int16_t>((uint16_t(data[1]) << 8) | uint16_t(data[0]));
        }

        static int32_t  read_int32_t(const uint8_t* data)
        {
            return static_cast<int32_t>(uint32_t(data[3]) << 24 | uint32_t(data[2]) << 16 | uint32_t(data[1] <<  8) | uint32_t(data[0]));
        }

        static uint16_t read_uint16_t(const uint8_t* data)
        {
            return (uint16_t(data[1]) << 8) | uint16_t(data[0]);
        }

        static uint32_t read_uint32_t(const uint8_t* data)
        {
            return uint32_t(data[3]) << 24 | uint32_t(data[2]) << 16 | uint32_t(data[1] <<  8) | uint32_t(data[0]);
        }

    bno_err_t parse_channel();

        bno_err_t parse_advertisement_package();

        bno_err_t parse_channel_control();

            bno_err_t parse_product_id_response(bno_product_id_t&dest, uint8_t report_length);

            bno_err_t parse_command_response();

                void parse_command_metadata(command_metadata_t& dest);

                bno_err_t parse_command_initialize(command_initialized_t& dest, uint8_t report_length);

                bno_err_t parse_command_me_calibration_response(command_me_calibration_config_t& dest, uint8_t report_length);

                bno_err_t parse_command_get_oscillator_type_response(command_oscillator_typte_t& dest, uint8_t report_length);

            bno_err_t parse_frs_write_response(bno_frs_write_response_t& dest, uint8_t report_length);

            bno_err_t parse_frs_read_response(bno_frs_read_response_t& dest, uint8_t report_length);

            bno_err_t parse_get_feature_response(bno_sensor_config_t& dest, uint8_t report_length);

        bno_err_t parse_channel_executable();

        bno_err_t parse_channel_reports();

            void parse_report_metadata(sensor_metadata_t& dest);

            bno_err_t parse_env_sensor(env_sensor_t& dest, uint8_t report_length, uint8_t data_bytes, float scale_factor);

            bno_err_t parse_quaternion(quaternion_t& dest, uint8_t report_length);

            bno_err_t parse_calibrated_accel_gyro_mag(accel_gyro_mag_t& dest, uint8_t report_length, float scale_factor);

            bno_err_t parse_uncalibrated_gyro_mag(uncalibrated_gyro_mag_t& dest, uint8_t report_length, float scale_factor);

            bno_err_t parse_raw_accel_gyro_mag(raw_accel_gyro_mag_t& dest, uint8_t report_length);

            bno_err_t parse_stability_classifier(stability_classifier_t& dest, uint8_t report_length);

            bno_err_t parse_stability_detector(stability_detector_t& dest, uint8_t report_length);

            bno_err_t parse_significant_motion_detector(significant_motion_detector_t& dest, uint8_t report_length);

            bno_err_t parse_step_counter(step_counter_t& dest, uint8_t report_length);

            bno_err_t parse_step_detector(step_counter_t& dest, uint8_t report_length);

        bno_err_t parse_channel_gyro();
};