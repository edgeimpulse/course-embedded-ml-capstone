/**
 * The code below can be used for inference in Arduino
 * 
 * TODO: IMU in ISR hangs processor!
 */

// These must be placed before #include "NRF52_MBED_TimerInterrupt.h"
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     3

// Include the name of the Edge Impulse SDK library you imported. This will 
// switch between Arduino and computer libraries as needed.
#ifdef ARDUINO
    #include <Arduino_LSM9DS1.h>
    #include <magic-wand-capstone_inferencing.h>
    #include <NRF52_MBED_TimerInterrupt.h>
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
#define SAMPLING_PERIOD_US  1000 * SAMPLING_PERIOD_MS   // Sampling period (us)
#define NUM_CHANNELS        EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME // 6 channels
#define NUM_READINGS        EI_CLASSIFIER_RAW_SAMPLE_COUNT      // 100 readings
#define NUM_CLASSES         EI_CLASSIFIER_LABEL_COUNT           // 4 classes

// Define the number of times inference happens each full window (1 second)
#define SLICES_PER_WINDOW   4                           // Inferences per sec

// Raw buffer (half of the double buffer) should be big enough for 1 slice
// This is also the "number of readings per slice"
#define RAW_BUF_SIZE        (NUM_CHANNELS * NUM_READINGS) / SLICES_PER_WINDOW

// Function declarations
static int get_signal_data(size_t offset, size_t length, float *out_ptr);
void timer_ISR();

// Means and standard deviations from our dataset curation
static const float means[] = {0.4869, -0.6364, 8.329, -0.1513, 4.631, -9.8836};
static const float std_devs[] = {3.062, 7.2209, 6.9951, 61.3324, 104.1638, 108.3149};

// Double buffer (used to capture raw samples from sensor)
static float raw_buf_0[RAW_BUF_SIZE];
static float raw_buf_1[RAW_BUF_SIZE];
static float *raw_buf_wr;
static float *raw_buf_rd;
static int raw_buf_count = 0;
static bool raw_buf_ready = false;

// Buffer that contains a full window for inference. Note that this is now a 
// ring buffer where older samples are overwritten!
static float input_buf[NUM_CHANNELS * NUM_READINGS];
static int input_buf_slice = 0;

// Wrapper for raw input buffer
static signal_t sig;

// Initialize NRF52 timer (only use NRF_TIMER_3 or NRF_TIMER_4)
NRF52_MBED_Timer interrupt_timer(NRF_TIMER_4);

// Interupt service routine
void timer_ISR() {
    float acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;

    // Get raw readings from the sensors
    IMU.readAcceleration(acc_x, acc_y, acc_z);
    IMU.readGyroscope(gyr_x, gyr_y, gyr_z);

    // Store the raw readings in the buffer (use the write pointer)
    raw_buf_wr[raw_buf_count + 0] = acc_x;
    raw_buf_wr[raw_buf_count + 1] = acc_y;
    raw_buf_wr[raw_buf_count + 2] = acc_z;
    raw_buf_wr[raw_buf_count + 3] = gyr_x;
    raw_buf_wr[raw_buf_count + 4] = gyr_y;
    raw_buf_wr[raw_buf_count + 5] = gyr_z;

    // Increment the counter by the number of readings you stored
    raw_buf_count += 6;

    // Swap pointers if buffer is full
    if (raw_buf_count >= RAW_BUF_SIZE) {
        raw_buf_count = 0;
        raw_buf_ready = true;
        if (raw_buf_wr == &raw_buf_0[0]) {
            raw_buf_wr = raw_buf_1;
            raw_buf_rd = raw_buf_0;
        } else {
            raw_buf_wr = raw_buf_0;
            raw_buf_rd = raw_buf_1;
        }
    }
}

// Setup function that is called once as soon as the program starts
void setup() {

    // Start serial port (Arduino only)
#ifdef ARDUINO
    Serial.begin(115200);
#endif

    // Initialize double buffer pointers
    raw_buf_wr = raw_buf_0;
    raw_buf_rd = raw_buf_1;

    // Clear ring buffer
    memset(input_buf, 0, (NUM_CHANNELS * NUM_READINGS) * sizeof(float));

    // Start accelerometer (part of IMU)
    if (!IMU.begin()) {
        ei_printf("ERROR: Failed to initialize IMU!\r\n");
        while (1);
    }
    
    // Configure interrupt (starts the timer to trigger ISR every 10 ms)
    if (!interrupt_timer.attachInterruptInterval(SAMPLING_PERIOD_US, timer_ISR)) {
        ei_printf("ERROR: Could not set timer interrupt\r\n");
    } else {
        ei_printf("ERROR: Could not set up timer\r\n");
    }

    // Assign callback function to fill buffer used for preprocessing/inference
    sig.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    sig.get_data = &get_signal_data;
}

// Loop function that is called repeatedly after setup()
void loop() {

    float acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
    ei_impulse_result_t result; // Used to store inference output
    EI_IMPULSE_ERROR res;       // Return code from inference
    int start_slice_offset;     // Index of the current slice in input_buf

    // If buffer is already full, it has been overrun
    if (raw_buf_ready) {
        ei_printf("ERROR: Buffer overrun\r\n");
        return;
    }

    // Wait until buffer is full
    while (!raw_buf_ready) {
        delay(10);
    }
    raw_buf_ready = false;

    // Compute the index of the current slice for input_buf
    start_slice_offset = RAW_BUF_SIZE * input_buf_slice;

    // Transform and copy contents of raw (read) buffer to input (ring) buffer
    for (int i = 0; i < (RAW_BUF_SIZE / NUM_CHANNELS); i++) {

        // Get accelerometer and gyroscope data from raw (read) buffer
        acc_x = raw_buf_rd[(NUM_CHANNELS * i) + 0];
        acc_y = raw_buf_rd[(NUM_CHANNELS * i) + 1];
        acc_z = raw_buf_rd[(NUM_CHANNELS * i) + 2];
        gyr_x = raw_buf_rd[(NUM_CHANNELS * i) + 3];
        gyr_y = raw_buf_rd[(NUM_CHANNELS * i) + 4];
        gyr_z = raw_buf_rd[(NUM_CHANNELS * i) + 5];

        // Convert accelerometer units from G to m/s^s
        acc_x *= CONVERT_G_TO_MS2;
        acc_y *= CONVERT_G_TO_MS2;
        acc_z *= CONVERT_G_TO_MS2;

        // Perform standardization on each reading
        // Use the values from means[] and std_devs[]
        acc_x = (acc_x - means[0]) / std_devs[0];
        acc_y = (acc_y - means[1]) / std_devs[1];
        acc_z = (acc_z - means[2]) / std_devs[2];
        gyr_x = (gyr_x - means[3]) / std_devs[3];
        gyr_y = (gyr_y - means[4]) / std_devs[4];
        gyr_z = (gyr_z - means[5]) / std_devs[5];

        // Fill the correct slice in input_buf with the standardized readings
        input_buf[start_slice_offset + (NUM_CHANNELS * i) + 0] = acc_x;
        input_buf[start_slice_offset + (NUM_CHANNELS * i) + 1] = acc_y;
        input_buf[start_slice_offset + (NUM_CHANNELS * i) + 2] = acc_z;
        input_buf[start_slice_offset + (NUM_CHANNELS * i) + 3] = gyr_x;
        input_buf[start_slice_offset + (NUM_CHANNELS * i) + 4] = gyr_y;
        input_buf[start_slice_offset + (NUM_CHANNELS * i) + 5] = gyr_z;
    }

    // Increment and wrap slice counter
    input_buf_slice++;
    if (input_buf_slice >= SLICES_PER_WINDOW) {
        input_buf_slice = 0;
    }

    // Call run_classifier() to perform preprocessing and inferece
    res = run_classifier(&sig, &result, false);

    // Find the label with the highest classification value
    float max_val = 0.0;
    int max_idx = -1;
    for (int i = 0; i < NUM_CLASSES; i++) {
        if (result.classification[i].value > max_val) {
            max_val = result.classification[i].value;
            max_idx = i;
        }
    }

    // Print return code and how long it took to perform inference
    ei_printf("run_classifier returned: %d\r\n", res);
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n", 
            result.timing.dsp, 
            result.timing.classification, 
            result.timing.anomaly);

    // Print inference/prediction results
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }


    // Print the answer (line must begin with "ANS: " for the autograder)
    ei_printf("ANS: %s, %f\r\n", 
                ei_classifier_inferencing_categories[max_idx], 
                result.classification[max_idx].value);

    //***TEST***
    static int slice_counter = 0;
    slice_counter++;
    if (slice_counter >= SLICES_PER_WINDOW) {
        slice_counter = 0;
        ei_printf("^^^\r\n");
    }
}

// Callback: fill a section of the out_ptr buffer when requested.
// Note that we must now start at the correct slice in the ring buffer.
static int get_signal_data(size_t offset, size_t length, float *out_ptr) {

    // Find where to start reading from the ring buffer
    size_t idx = offset + (input_buf_slice * RAW_BUF_SIZE);

    // Copy the elements in the ring buffer to the output buffer
    for (size_t i = 0; i < length; i++) {

        // Copy element
        out_ptr[i] = input_buf[idx];     

        // Increment and wrap the ring buffer pointer
        idx++;
        if (idx >= (NUM_CHANNELS * NUM_READINGS)) {
            idx = offset;
        }
    }


    // // ***TEST*** Print buffer in inference order
    // int timestamp = 0; // ***TEST***
    // int counter = 0; // ***TEST***
    // for (int i = 0; i < length / 6; i++) {
    //     ei_printf("%i, ", timestamp);
    //     timestamp += 10;
    //     for (int j = 0; j < 5; j++) {
    //         ei_printf("%.2f, ", out_ptr[(i * 6) + j]);
    //     }
    //     ei_printf("%.2f\r\n", out_ptr[(i * 6) + 5]);
    // }

    return EIDSP_OK;
}