#include <iostream>
#include <thread>
#include <unistd.h>
#include <mutex>
#include <condition_variable>
#include <stdio.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "radar_hal.h"
#include "taskRadar.h"
#include "x4driver.h"
#include "xep_hal.h"
#include "../hal/radar_hal.h"
#include <linux/input.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>


volatile xtx4driver_errors_t x4_initialize_status = XEP_ERROR_X4DRIVER_UNINITIALIZED;
X4Driver_t *x4driver = NULL;

#define DEBUG 0

using namespace std;
std::recursive_mutex x4driver_mutex;

typedef struct
{
    radar_handle_t *radar_handle; // Some info separating different radar chips on the same module.
} XepRadarX4DriverUserReference_t;

typedef struct
{
    X4Driver_t *x4driver;
} RadarTaskParameters_t;

void x4driver_GPIO_init(void)
{
    wiringPiSetup();
    pinMode(X4_ENABLE_PIN, OUTPUT);
    pinMode(X4_GPIO_INT, INPUT);
    pullUpDnControl(X4_GPIO_INT, PUD_DOWN);
}

void x4driver_spi_init(void)
{
    wiringPiSPISetup(SPI_CHANNEL, 32000000);
}

uint32_t frame_counter = 0;
void x4driver_data_ready(void)
{

    uint32_t status = XEP_ERROR_X4DRIVER_OK;
    uint32_t bin_count = 0;
    x4driver_get_frame_bin_count(x4driver, &bin_count);
    uint8_t down_conversion_enabled = 0;
    x4driver_get_downconversion(x4driver, &down_conversion_enabled);

    uint32_t fdata_count = bin_count;
    if (down_conversion_enabled == 1)
    {
        fdata_count = bin_count * 2;
    }

    float32_t data_frame_normalized[fdata_count];

    status = x4driver_read_frame_normalized(x4driver, &frame_counter, data_frame_normalized, fdata_count);

    if (XEP_ERROR_X4DRIVER_OK == status)
    {

        printf("frame :%d.\n", frame_counter);

        char *filename = "1.csv";
        FILE *f = fopen(filename, "a");
        if (f == NULL)
            return;

        for (int i = 0; i < (sizeof(data_frame_normalized) / sizeof(data_frame_normalized[0])); i++)
        {
            fprintf(f, "%lf%s", data_frame_normalized[i], (i<fdata_count-1?",":""));
        }

        fprintf(f, "\n");
        fclose(f);
    }
    else
    {
        printf("frame error.\n");
    }


}

static uint32_t x4driver_callback_take_sem(void *sem, uint32_t timeout)
{
    x4driver_mutex.lock();
    return 1;
}

static void x4driver_callback_give_sem(void *sem)
{
    x4driver_mutex.unlock();
}

static uint32_t x4driver_callback_pin_set_enable(void *user_reference, uint8_t value)
{
    XepRadarX4DriverUserReference_t *x4driver_user_reference = (XepRadarX4DriverUserReference_t *)user_reference;
    int status = radar_hal_pin_set_enable(x4driver_user_reference->radar_handle, value);
    return status;
}

static uint32_t x4driver_callback_spi_write(void *user_reference, uint8_t *data, uint32_t length)
{
    XepRadarX4DriverUserReference_t *x4driver_user_reference = (XepRadarX4DriverUserReference_t *)user_reference;
    return radar_hal_spi_write(x4driver_user_reference->radar_handle, data, length);
}
static uint32_t x4driver_callback_spi_read(void *user_reference, uint8_t *data, uint32_t length)
{
    XepRadarX4DriverUserReference_t *x4driver_user_reference = (XepRadarX4DriverUserReference_t *)user_reference;
    return radar_hal_spi_read(x4driver_user_reference->radar_handle, data, length);
}

static uint32_t x4driver_callback_spi_write_read(void *user_reference, uint8_t *wdata, uint32_t wlength, uint8_t *rdata, uint32_t rlength)
{
    XepRadarX4DriverUserReference_t *x4driver_user_reference = (XepRadarX4DriverUserReference_t *)user_reference;
    return radar_hal_spi_write_read(x4driver_user_reference->radar_handle, wdata, wlength, rdata, rlength);
}

static void x4driver_callback_wait_us(uint32_t us)
{
    delayMicroseconds(us);
}

void x4driver_enable_ISR(void *user_reference, uint32_t enable)
{
    if (enable == 1)
    {
        pinMode(X4_GPIO_INT, INPUT);
        pullUpDnControl(X4_GPIO_INT, PUD_DOWN);
        if (wiringPiISR(X4_GPIO_INT, INT_EDGE_RISING, &x4driver_data_ready) < 0)
        {
            printf("unable to setup ISR");
        }
    }
    else
        pinMode(X4_GPIO_INT, OUTPUT); //disable Interrupt
}

uint32_t task_radar_init(X4Driver_t **x4driver)
{
    x4driver_GPIO_init();
    x4driver_spi_init();

    XepRadarX4DriverUserReference_t *x4driver_user_reference = (XepRadarX4DriverUserReference_t *)malloc(sizeof(XepRadarX4DriverUserReference_t));
    memset(x4driver_user_reference, 0, sizeof(XepRadarX4DriverUserReference_t));

    void *radar_hal_memory = malloc(radar_hal_get_instance_size());
    int status = radar_hal_init(&(x4driver_user_reference->radar_handle), radar_hal_memory);

    X4DriverLock_t lock;
    lock.object = (void *)&x4driver_mutex;
    lock.lock = x4driver_callback_take_sem;
    lock.unlock = x4driver_callback_give_sem;

    uint32_t timer_id_sweep = 2;
    X4DriverTimer_t timer_sweep;

    X4DriverTimer_t timer_action;

    X4DriverCallbacks_t x4driver_callbacks;

    x4driver_callbacks.pin_set_enable = x4driver_callback_pin_set_enable; // X4 ENABLE pin
    x4driver_callbacks.spi_read = x4driver_callback_spi_read;             // SPI read method
    x4driver_callbacks.spi_write = x4driver_callback_spi_write;           // SPI write method
    x4driver_callbacks.spi_write_read = x4driver_callback_spi_write_read; // SPI write and read method
    x4driver_callbacks.wait_us = x4driver_callback_wait_us;               // Delay method
                                                                          //  x4driver_callbacks.notify_data_ready = x4driver_notify_data_ready;      // Notification when radar data is ready to read
                                                                          //  x4driver_callbacks.trigger_sweep = x4driver_trigger_sweep_pin;          // Method to set X4 sweep trigger pin
    x4driver_callbacks.enable_data_ready_isr = x4driver_enable_ISR;       // Control data ready notification ISR

    void *x4driver_instance_memory = malloc(x4driver_get_instance_size()); //pvPortMalloc(x4driver_get_instance_size());
    x4driver_create(x4driver, x4driver_instance_memory, &x4driver_callbacks, &lock, &timer_sweep, &timer_action, x4driver_user_reference);


    RadarTaskParameters_t *task_parameters = (RadarTaskParameters_t *)malloc(sizeof(RadarTaskParameters_t));
    //task_parameters->dispatch = dispatch;
    task_parameters->x4driver = *x4driver;

    task_parameters->x4driver->spi_buffer_size = 192 * 32;
    task_parameters->x4driver->spi_buffer = (uint8_t *)malloc(task_parameters->x4driver->spi_buffer_size);
    if ((((uint32_t)task_parameters->x4driver->spi_buffer) % 32) != 0)
    {
        int alignment_diff = 32 - (((uint32_t)task_parameters->x4driver->spi_buffer) % 32);
        task_parameters->x4driver->spi_buffer += alignment_diff;
        task_parameters->x4driver->spi_buffer_size -= alignment_diff;
    }
    task_parameters->x4driver->spi_buffer_size -= task_parameters->x4driver->spi_buffer_size % 32;

    return XT_SUCCESS;
}

int taskRadar(void)
{
    int xgl_iter = 16;
    int xgl_pulse_per_step = 1;
    int xgl_dacmin = 949;  //949;
    int xgl_dacmax = 1100; //1100
    xtx4_dac_step_t xgl_dacstep = DAC_STEP_1;
    float xgl_fps = 40;
    int xgl_getiq = 1;

    printf("task_radar start!\n");

    uint32_t status = 0;

    status = task_radar_init(&x4driver);

    FILE *f;

    f = fopen("1.csv", "w");
    fclose(f);

    xtx4driver_errors_t tmp_status = (xtx4driver_errors_t)x4driver_init(x4driver);



    // if (status != XEP_ERROR_X4DRIVER_OK)
    // {
    //     printf("Error setting dac minimum\n");
    //     printf("Error code=%d\n", status);
    //     return 1;
    // }

    //     TX_CENTER_FREQUENCY_MIN                     = 2,
    //     TX_CENTER_FREQUENCY_EU_7_290GHz             = 3,
    //     TX_CENTER_FREQUENCY_KCC_8_748GHz            = 4,
    //     TX_CENTER_FREQUENCY_MAX                     = 5,

    status = x4driver_set_sweep_trigger_control(x4driver, SWEEP_TRIGGER_X4); //SWEEP_TRIGGER_MANUAL// MANUAL By default let sweep trigger control done by X4
    status = x4driver_set_tx_center_frequency(x4driver, (xtx4_tx_center_frequency_t)4);
    status = x4driver_set_dac_min(x4driver, xgl_dacmin);
    status = x4driver_set_dac_max(x4driver, xgl_dacmax);
    status = x4driver_set_iterations(x4driver, xgl_iter); //32);
    status = x4driver_set_pulses_per_step(x4driver, xgl_pulse_per_step); //140);
    status = x4driver_set_downconversion(x4driver, xgl_getiq); //1);// Radar data as downconverted baseband IQ, not RF.
    status = x4driver_set_frame_area_offset(x4driver, 0); // Given by module HW. Makes frame_area start = 0 at front of module.
    status = x4driver_set_frame_area(x4driver, 0.0, 7.2); // Observe from 0.5m to 4.0m.
    status = x4driver_check_configuration(x4driver);
    status = x4driver_set_dac_step(x4driver, xgl_dacstep);
    status = x4driver_set_fps(x4driver, xgl_fps); /***************set fps, this will trigger data output***************/
    status = x4driver_check_configuration(x4driver);

    x4driver_start_sweep(x4driver);

    while(true){
        
    }
}
