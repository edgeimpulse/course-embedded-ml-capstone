/**
 * The code below can be used for inference in Arduino
 */

// Include the name of the Edge Impulse SDK library you imported. This will 
// switch between Arduino and computer libraries as needed.
#ifdef ARDUINO
    #include <magic-wand-capstone_inferencing.h>
#else
    #include "time-emulator.h"
    #include "imu-emulator.h"
    #include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#endif

// Function declarations
static int get_signal_data(size_t offset, size_t length, float *out_ptr);

// Settings
static const int debug_nn = false;

// Raw features copied from test sample (Edge Impulse > Model testing)
static float input_buf[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// Wrapper for raw input buffer
static signal_t sig;

// Setup function that is called once as soon as the program starts
void setup() {

    // Start serial port (Arduino only)
#ifdef ARDUINO
    Serial.begin(115200);
#endif

    // Print something to the terminal
    ei_printf("Sequential inference test\r\n");

    // Start accelerometer (part of IMU)
    if (!IMU.begin()) {
        ei_printf("ERROR: Failed to initialize IMU!\r\n");
        while (1);
    }

    // TEST
    // float x = 0;
    // float y = 0;
    // float z = 0;
    // IMU.readAcceleration(x, y, z);
    // printf("%f %f %f\r\n", x, y, z);
    // IMU.readGyroscope(x, y, z);
    // printf("%f %f %f\r\n", x, y, z);

    // Assign callback function to fill buffer used for preprocessing/inference
    sig.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    sig.get_data = &get_signal_data;
}

// Loop function that is called repeatedly after setup()
void loop() {

    ei_impulse_result_t result; // Used to store inference output
    EI_IMPULSE_ERROR res;       // Return code from inference
    float acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;

    // Sample from sensors to fill buffer at given sample rate
    for (int i = 0; i < 10; i++) {
        IMU.readAcceleration(acc_x, acc_y, acc_z);
        IMU.readGyroscope(gyr_x, gyr_y, gyr_z);
        printf("Accel: %f %f %f\r\n", acc_x, acc_y, acc_z);
        printf("Gyro: %f %f %f\r\n", gyr_x, gyr_y, gyr_z);
        delay(10);
    }

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