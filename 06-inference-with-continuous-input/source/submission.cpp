/**
 * The code below can be used for inference in Arduino
 */

// Include the name of the Edge Impulse SDK library you imported. This will 
// switch between Arduino and computer libraries as needed.
#ifdef ARDUINO
    #include <Arduino_LSM9DS1.h>
    #include <magic-wand-capstone_inferencing.h>
#else
    #include "time-emulator.h"
    #include "imu-emulator.h"
    #include "nrf52-timer-emulator.h"
    #include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#endif

// Constants
#define CONVERT_G_TO_MS2    9.80665f  // Used to convert G to m/s^2
#define SAMPLING_FREQ_HZ    EI_CLASSIFIER_FREQUENCY     // 100 Hz sampling rate
#define SAMPLING_PERIOD_MS  1000 / SAMPLING_FREQ_HZ     // Sampling period (ms)
#define NUM_CHANNELS        EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME // 6 channels
#define NUM_READINGS        EI_CLASSIFIER_RAW_SAMPLE_COUNT      // 100 readings
#define NUM_CLASSES         EI_CLASSIFIER_LABEL_COUNT           // 4 classes

// Define the amount of time (in microseconds) between ISR calls
#define TIMER_INTERVAL_US   10000    // Time between ISR calls (microseconds)

// Function declarations
static int get_signal_data(size_t offset, size_t length, float *out_ptr);
void timer_ISR();

// Means and standard deviations from our dataset curation
static const float means[] = {0.4869, -0.6364, 8.329, -0.1513, 4.631, -9.8836};
static const float std_devs[] = {3.062, 7.2209, 6.9951, 61.3324, 104.1638, 108.3149};

// Raw features copied from test sample (Edge Impulse > Model testing)
static float input_buf[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// Wrapper for raw input buffer
static signal_t sig;

// Initialize NRF52 timer (only use NRF_TIMER_3 or NRF_TIMER_4)
NRF52_MBED_Timer interrupt_timer(NRF_TIMER_4);

// Interupt service routine
void timer_ISR() {
  ei_printf("TEST\r\n");
}

// Setup function that is called once as soon as the program starts
void setup() {

    // Start serial port (Arduino only)
#ifdef ARDUINO
    Serial.begin(115200);
#endif

    // Start accelerometer (part of IMU)
    if (!IMU.begin()) {
        ei_printf("ERROR: Failed to initialize IMU!\r\n");
        while (1);
    }
    
    // Configure interrupt
    if (!interrupt_timer.attachInterruptInterval(TIMER_INTERVAL_US, timer_ISR)) {
        ei_printf("ERROR: Could not set timer interrupt");
    }

    // Assign callback function to fill buffer used for preprocessing/inference
    sig.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    sig.get_data = &get_signal_data;
}

// Loop function that is called repeatedly after setup()
void loop() {

    float acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
    unsigned long timestamp;
    ei_impulse_result_t result; // Used to store inference output
    EI_IMPULSE_ERROR res;       // Return code from inference

    // Sample from sensors to fill buffer at given sample rate
    for (int i = 0; i < 10; i++) {
        IMU.readAcceleration(acc_x, acc_y, acc_z);
        IMU.readGyroscope(gyr_x, gyr_y, gyr_z);
        // printf("Accel: %f %f %f\r\n", acc_x, acc_y, acc_z);
        // printf("Gyro: %f %f %f\r\n", gyr_x, gyr_y, gyr_z);
        delay(10);
    }

    //*** TODO ***
    // - Create interrupt service routine in Arduino and computer (callback?)
    // - ISR should convert acc to MS2 and perform normalization
    // - ISR(?) should shift working buffer and append new samples
    // - Perform inference every x ms (100 ms?)
    //      - Working buffer is copied to inference buffer before each inference
    // - Maybe demonstrate run_classifier_continuous()? Probably not, though

    // // Perform DSP pre-processing and inference
    // res = run_classifier(&sig, &result, false);

    // // Print return code and how long it took to perform inference
    // ei_printf("run_classifier returned: %d\r\n", res);
    // ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n", 
    //         result.timing.dsp, 
    //         result.timing.classification, 
    //         result.timing.anomaly);

    // // Print inference/prediction results
    // ei_printf("Predictions:\r\n");
    // for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    //     ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
    //     ei_printf("%.5f\r\n", result.classification[i].value);
    // }

    // // Wait 100 ms before running inference again
    // ei_sleep(100);
}

// Callback: fill a section of the out_ptr buffer when requested
static int get_signal_data(size_t offset, size_t length, float *out_ptr) {
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = (input_buf + offset)[i];
    }

    return EIDSP_OK;
}