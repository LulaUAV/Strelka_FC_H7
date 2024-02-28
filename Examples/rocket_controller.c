/*
File name: rocket_controller.h
Author: Angus McLennan
Created on: Feb 28, 2024
Description: An example for usage of the State_Controller library
THIS CODE IS A GUIDE. It has not been tested in flight. Any implementation using this code must be tested thoroughly before use in a mission critical application

Copyright © 2024 Angus McLennan

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdint.h>
#include <stdbool.h>
#include "State_Controller.h"

// Define variable to set flag to be true when sensors are initialised
bool sensors_initialised = false;

void main()
{
    while (!sensors_initialised)
    {
        osDelay(10);
    }

    // Check continuity of e-matches and report error if fault detected
    /*
    TODO: Insert code here
    */

    float ax, ay, az, rocket_angle, altitude;
    uint32_t timestamp_ms;

    // Initialise State_Controller
    altitude = 0; // TODO: Read barometric altitude
    if (init_state_controller(altitude))
    {
        // Handle error state
    }

    // Define launch detection loop
    ax = 0; // TODO: Read accelerometer X axis
    ay = 0; // TODO: Read accelerometer Y axis
    az = 0; // TODO: Read accelerometer Z axis
    /*
        TODO: Calculate rocket angle.
        Accelerometer can be used to find gravity vector. Averaging or low pass filtering may be required since the reading will be noisy.
        IMPORTANT: The result from the rocket angle calculation should be close to zero when the rocket is pointing up. Given that the orientation of the flight computer
        is application dependent therefore, the selection of which accelerometer axes are used to determine the gravity vector must be chosen carefully.
        If you wish to disable the vehicle orientation check, set the rocket_angle to zero. The rocket must not be disturbed once armed in any orientation.
    */
    rocket_angle = 0;
    timestamp_ms = millis(); // TODO: Read system time in miliseconds
    bool launch_detected = detect_launch_accel(ax, ay, az, rocket_angle, timestamp_ms);
    while (!launch_detected)
    {
        // Read sensors
        ax = 0;                  // TODO: Read accelerometer X axis
        ay = 0;                  // TODO: Read accelerometer Y axis
        az = 0;                  // TODO: Read accelerometer Z axis
        rocket_angle = 0;        // TODO: Calculate rocket angle
        timestamp_ms = millis(); // TODO: Read system time in miliseconds
        // Update launch detected variable
        launch_detected = detect_launch_accel(ax, ay, az, rocket_angle, timestamp_ms);

        // Delay 1ms
        osDelay(1);
    }

    // Launch has been detected
    // Determine which axis is up
    ax = 0; // TODO: Read accelerometer X axis
    ay = 0; // TODO: Read accelerometer Y axis
    az = 0; // TODO: Read accelerometer Z axis
    while (!calculate_up_axis(ax, ay, az))
    {
        ax = 0; // TODO: Read accelerometer X axis
        ay = 0; // TODO: Read accelerometer Y axis
        az = 0; // TODO: Read accelerometer Z axis
        osDelay(10);
    }

    // Define burnout and apogee detection loop
    bool apogee_detected = false;
    while (!apogee_detected)
    {
        ax = 0;                  // TODO: Read accelerometer X axis
        ay = 0;                  // TODO: Read accelerometer Y axis
        az = 0;                  // TODO: Read accelerometer Z axis
        altitude = 0;            // TODO: Read barometric altitude
        timestamp_ms = millis(); // TODO: Read system time in miliseconds
        // Detect burnout state
        detect_burnout_accel(ax, ay, az, altitude, timestamp_ms);
        // Detect if apogee has been reached
        apogee_detected = detect_apogee(ax, ay, az, altitude, timestamp_ms);

        // Delay 1ms
        osDelay(1);
    }

    // TODO: Deploy drogue parachute

    // Define main deployment altitude detection loop
    bool main_deploy_altitude = false;
    while (!main_deploy_altitude)
    {
        altitude = 0;            // TODO: Read barometric altitude
        timestamp_ms = millis(); // TODO: Read system time in miliseconds
        main_deploy_altitude = detect_main_deploy_altitude(altitude, timestamp_ms);
    }

    // TODO: Deploy main parachutes

    // Define landing detection loop
    bool landing_detected = false;
    while (!landing_detected)
    {
        altitude = 0;            // TODO: Read barometric altitude
        timestamp_ms = millis(); // TODO: Read system time in miliseconds
        landing_detected = detect_landing(altitude, timestamp_ms);
    }

    // Rocket has landed

    // Enter idle loop
    while (1)
    {
        osDelay(1000);
    }
}