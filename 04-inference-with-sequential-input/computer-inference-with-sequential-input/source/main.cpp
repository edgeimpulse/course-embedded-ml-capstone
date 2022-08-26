    // TODO: 
    // - make user submission a separate file (submission.c) with an
    //   associated interface (submission.h). Link to main.cpp.
    // - Create IMU class with .begin(), .readAcceleration(), and .readGyroscope()
    //   methods that fetch raw values from vector
    // - Change test cases to RAW (not standardized)
    // - Modify while(1) loop; to be conditioned on number of argument test files
    //   so that it's only called as long as there's a test file in the queue
    // - Student should ei_printf the inference values?
    // - Create Python script that calls app with .csv files and compares output
    //   to known good answers

#include <stdio.h>
#include <array>

#include "csv.h"
#include "imu-emulator.h"
#include "submission.h"

// Read accelerometer callback function
int readAccelerometerCallback(float& x, float& y, float& z) {
    x = 1;
    y = 2;
    z = 3512;

    return 1;
}

// Read gyroscope callback function
int readGyroscopeCallback(float& x, float& y, float& z) {
    x = 10;
    y = 20;
    z = 12345;

    return 1;
}

// Main function to call setup and loop
int main(int argc, char **argv) {

    float timestamp, accX, accY, accZ, gyrX, gyrY, gyrZ;
    std::array<float, 7> reading;
    std::vector<std::array<float, 7>> raw_readings;
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
            reading[0] = timestamp;
            reading[1] = accX;
            reading[2] = accY;
            reading[3] = accZ;
            reading[4] = gyrX;
            reading[5] = gyrY;
            reading[6] = gyrZ;

            // Push array onto vector
            raw_readings.push_back(reading);

            // Increment our index
            reading_idx++;
        }
    }

    // // TEST
    // for (int i = 0; i < 7; i++) {
    //     float & val = raw_readings[99][i];
    //     printf("%f\r\n", val);
    // }

    // Register the callback functions to simulate reading from the IMU
    IMU.registerAccelCallback(readAccelerometerCallback);
    IMU.registerGyroCallback(readGyroscopeCallback);

    // Run user submission
    setup();
    // while (1) {
    //     loop();
    // }

    return 0;
}

