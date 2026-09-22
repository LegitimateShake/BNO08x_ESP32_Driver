#include <stdio.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <bno08x.h>

/* Change these parameters to the parameters of your setup */
static constexpr uint8_t I2C_ADDRESS = bno_constants::i2c::DEFAULT_ADDRESS;
static constexpr uint32_t I2C_CLOCK_SPEED = 400'000;

static constexpr gpio_num_t PIN_SCL = GPIO_NUM_14;
static constexpr gpio_num_t PIN_SDA = GPIO_NUM_13;
static constexpr gpio_num_t PIN_RST = GPIO_NUM_15;
static constexpr gpio_num_t PIN_INT = GPIO_NUM_16;

/* 
    Create a new BNO08x object on the heap 
    The object is rather large, so not placing
    it on the stack is adviced
*/
BNO08x* bno = new BNO08x;

extern "C" void app_main()
{   
    i2c_master_bus_handle_t bus_handle;

    /* Here we create the sensor configuration struct */
    bno_config_t bno_init_cfg = {
        .i2c_address = I2C_ADDRESS,
        .rst = PIN_RST,
        .h_int = PIN_INT,
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
    bno_check_sc("Initialization", bno->begin(&bno_init_cfg));

    /* Create a configuration struct for a sensor of the BNO08x */
    bno_sensor_config_t sensor_cfg = bno->create_sensor_config(bno_sensor_id_t::ROTATION_VECTOR, 100);

    /* Instruct the BNO08x to enable the sensor, based on the configuration struct */
    bno_check_sc("Enable sensor", bno->enable_sensor(sensor_cfg));

    /* Data will be stored into this object */
    quaternion_t quat;

    while(true) {

        /* 
            Here we wait for sensor data This funciton will wait for 20ms. If no
            new data arrives in that timespan, the fuction will return a timeout
            and the program will continue. By setting the ticks_to_timeout to 0 
            this function will become non-blocking and check for new data only once 
        */
        if(bno->read_rotation_vector(quat, pdMS_TO_TICKS(20)) == bno_err_t::OK) {

            printf("%.2f | %.2f | %.2f | %.2f\n", quat.w, quat.x, quat.y, quat.z);
        }
    }
}


