// TODO: 
// - Change test cases to RAW (not standardized)
// - Modify while(1) loop; to be conditioned on number of argument test files
//   so that it's only called as long as there's a test file in the queue
// - Student should ei_printf the inference values?
// - Create Python script that calls app with .csv files and compares output
//   to known good answers

#include <stdio.h>
#include <cstdlib>
#include <array>

#include "csv.h"
#include "time-emulator.h"
#include "imu-emulator.h"
#include "submission.h"

// Declare our helper functions
int findClosestIdx(unsigned long time_ms);
int readAccelerometerCallback(float& x, float& y, float& z);
int readGyroscopeCallback(float& x, float& y, float& z);

// How the arrays in the raw readings vector are indexed
enum VectorIDXs {
    TIME_IDX = 0,
    ACC_X_IDX,
    ACC_Y_IDX,
    ACC_Z_IDX,
    GYR_X_IDX,
    GYR_Y_IDX,
    GYR_Z_IDX
};

// Vector of raw readings to be supplied to the user via callbacks
static std::vector<std::array<float, 7>> raw_readings;

// Current timestamp (milliseconds) of user's IMU readings
static unsigned long first_reading_timestamp = 0;
static bool is_first_reading = true;

/*******************************************************************************
 * Main
 */

// Main function to call setup and loop
int main(int argc, char **argv) {

    float timestamp, accX, accY, accZ, gyrX, gyrY, gyrZ;
    std::array<float, 7> reading;
    
    float sample_rate = 0.0;
    int reading_idx = 0;

    // Parse argument as input file path
    if (argc < 2) {
    printf("ERROR: No input file specified\r\n");
        return 1;
    }

    // Loop through all files provided as arguments
    for (int file_idx = 1; file_idx < argc; file_idx++) {

        // Read CSV header
        io::CSVReader<7> csv_reader(argv[file_idx]);
        csv_reader.read_header( io::ignore_extra_column, 
                                "timestamp", 
                                "accX", 
                                "accY",
                                "accZ",
                                "gyrX",
                                "gyrY",
                                "gyrZ");

        // Construct vector of raw values
        while (csv_reader.read_row(timestamp, accX, accY, accZ, gyrX, gyrY, gyrZ)) {

            // Calculate sample rate (and use that instead of what's in CSV)
            if (reading_idx == 0) {
                sample_rate = timestamp;
            } else if (reading_idx == 1) {
                sample_rate = timestamp - sample_rate;
            }
            timestamp = sample_rate * raw_readings.size();

            // Read values into array
            reading[TIME_IDX] = timestamp;
            reading[ACC_X_IDX] = accX;
            reading[ACC_Y_IDX] = accY;
            reading[ACC_Z_IDX] = accZ;
            reading[GYR_X_IDX] = gyrX;
            reading[GYR_Y_IDX] = gyrY;
            reading[GYR_Z_IDX] = gyrZ;

            // Push array onto vector
            raw_readings.push_back(reading);

            // Increment our index
            reading_idx++;
        }
    }

    // Register the callback functions to simulate reading from the IMU
    IMU.registerAccelCallback(readAccelerometerCallback);
    IMU.registerGyroCallback(readGyroscopeCallback);

    // // TEST
    // for (int i = 0; i < 7; i++) {
    //     float & val = raw_readings[99][i];
    //     printf("%f\r\n", val);
    // }



    // // Get monotonic clock time
    // unsigned long time_stamp = micros();

    // for (int i = 0; i < 10; i++) {
    //     delay(1000);
    //     printf("Elapsed ms: %lu\r\n", micros() - time_stamp);
    // }

    // Run user submission
    setup();
    while (1) {
        loop();
    }

    return 0;
}

/*******************************************************************************
 * Functions
 */

// Read accelerometer callback function
int readAccelerometerCallback(float& x, float& y, float& z) {

    // Update timestamp
    if (is_first_reading) {
        is_first_reading = false;
        first_reading_timestamp = millis();
    }

    // Calculated elapsed time
    unsigned long elapsed = millis() - first_reading_timestamp;
    int closest_time_idx = findClosestIdx(elapsed);

    // Assign values from the row closest to the requested elapsed timestamp
    x = raw_readings[closest_time_idx][ACC_X_IDX];
    y = raw_readings[closest_time_idx][ACC_Y_IDX];
    z = raw_readings[closest_time_idx][ACC_Z_IDX];

    return 1;
}

// Read gyroscope callback function
int readGyroscopeCallback(float& x, float& y, float& z) {

    // Update timestamp
    if (is_first_reading) {
        is_first_reading = false;
        first_reading_timestamp = millis();
    }

    // Calculated elapsed time
    unsigned long elapsed = millis() - first_reading_timestamp;
    int closest_time_idx = findClosestIdx(elapsed);

    // Assign values from the row closest to the requested elapsed timestamp
    x = raw_readings[closest_time_idx][GYR_X_IDX];
    y = raw_readings[closest_time_idx][GYR_Y_IDX];
    z = raw_readings[closest_time_idx][GYR_Z_IDX];

    return 1;
}

// Get closest reading from vector of readings
int findClosestIdx(unsigned long time_ms) {
    unsigned long closest_time = raw_readings[0][TIME_IDX];
    int closest_time_idx = 0;
    unsigned long num;

    for (long unsigned int i = 0; i < raw_readings.size(); i++) {
        num = raw_readings[i][TIME_IDX];
        if (abs(time_ms - num) < 
            abs(time_ms - closest_time)) {
                closest_time = num;
                closest_time_idx = i;
        }
    }

    return closest_time_idx;
}