#include <stdio.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <bno08x.h>

/* 
    Change these parameters to the parameters of your setup 
    This example demonstrates two sensors on the same I2C bus
*/
static constexpr uint8_t I2C_ADDRESS_SENSOR_1 = bno_constants::i2c::DEFAULT_ADDRESS;
static constexpr uint8_t I2C_ADDRESS_SENSOR_2 = bno_constants::i2c::ALTERNATE_ADDRESS;
static constexpr uint32_t I2C_CLOCK_SPEED = 400'000;

static constexpr gpio_num_t PIN_SCL = GPIO_NUM_14;
static constexpr gpio_num_t PIN_SDA = GPIO_NUM_13;

static constexpr gpio_num_t PIN_RST_1 = GPIO_NUM_15;
static constexpr gpio_num_t PIN_INT_1 = GPIO_NUM_16;

static constexpr gpio_num_t PIN_RST_2 = GPIO_NUM_6;
static constexpr gpio_num_t PIN_INT_2 = GPIO_NUM_5;

/* 
    Create two new BNO08x object on the heap 
    The object is rather large, so not placing
    it on the stack is adviced
*/
BNO08x* bno_1 = new BNO08x;
BNO08x* bno_2 = new BNO08x;

extern "C" void app_main()
{   
    i2c_master_bus_handle_t bus_handle;

    /* Here we create the sensor configuration struct */
    bno_config_t bno_init_cfg_1 = {
        .i2c_address = I2C_ADDRESS_SENSOR_1,
        .rst = PIN_RST_1,
        .h_int = PIN_INT_1,
        .bus_handle = &bus_handle,
        .i2c_frequency = I2C_CLOCK_SPEED
    };

    bno_config_t bno_init_cfg_2 = {
        .i2c_address = I2C_ADDRESS_SENSOR_2,
        .rst = PIN_RST_2,
        .h_int = PIN_INT_2,
        .bus_handle = &bus_handle,
        .i2c_frequency = I2C_CLOCK_SPEED
    };

    /* Here we create the I2C peripheral configuration struct */
    i2c_master_bus_config_t i2c_config = {
        .i2c_port = 0,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL, 
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 0,
            .allow_pd = 0
        }
    };

    /* Allocate the I2C bus */
    i2c_new_master_bus(&i2c_config, &bus_handle);

    /* 
        Error codes are checked through bno_check_sc(TAG, error_code) 
        If the error code is bno_err_t::OK, the fuction continues as 
        normal. Any other code blocks indefinetly                
    */

    /* Initialize the Object. MUST ALWAYS be called first */
    bno_check_sc("Initialization 1", bno_1->begin(&bno_init_cfg_1));
    bno_check_sc("Initialization 2", bno_2->begin(&bno_init_cfg_2));

    /* Create a configuration struct for a sensor of the BNO08x */
    bno_sensor_config_t sensor_cfg_1 = bno_1->create_sensor_config(bno_sensor_id_t::ROTATION_VECTOR, 100);
    bno_sensor_config_t sensor_cfg_2 = bno_2->create_sensor_config(bno_sensor_id_t::ACCELEROMETER, 100);

    /* Instruct the BNO08x to enable the sensor, based on the configuration struct */
    bno_check_sc("Enable sensor 1", bno_1->enable_sensor(sensor_cfg_1));
    bno_check_sc("Enable sensor 2", bno_2->enable_sensor(sensor_cfg_2));

    /* Data will be stored into this object */
    quaternion_t quat;
    accel_gyro_mag_t accel;

    while(true) {

        /* 
            Here we wait for sensor data This funciton will wait for 20ms. If no
            new data arrives in that timespan, the fuction will return a timeout
            and the program will continue. By setting the ticks_to_timeout to 0 
            this function will become non-blocking and check for new data only once 
        */
        if(bno_1->read_rotation_vector(quat, pdMS_TO_TICKS(20)) == bno_err_t::OK) {

            printf("Quaternion Sensor 1: %.2f | %.2f | %.2f | %.2f\n", quat.w, quat.x, quat.y, quat.z);
        }

        if(bno_2->read_acceleration(accel, pdMS_TO_TICKS(20)) == bno_err_t::OK) {

            printf("Acceleration Sensor 2: %.2f | %.2f | %.2f\n", accel.x, accel.y, accel.z);
        }
    }
}


