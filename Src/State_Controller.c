/*
 State_Controller.c

 Created on: Feb 27, 2024
 Author: Angus McLennan
 */

#include "State_Controller.h"

System_State_t system_state;
MedianFilter_t launch_median_filter;
MedianFilter_t burnout_median_filter;
MedianFilter_t apogee_median_filter;
ExpLowPassFilter_t altitude_exp_lp_filter;
ExpLowPassFilter_t accel_low_pass_filter;
State_Machine_Internal_State_t internal_state_fc;
ExpLowPassFilter_t landing_lp_filter;

uint32_t last_velocity_calculation_time = 0;
float last_altitude = 0;

uint8_t init_state_controller(float starting_altitude)
{
    system_state.flight_state = IDLE_ON_PAD;

    // Initialise launch median filter
    initMedianFilter(&launch_median_filter, LAUNCH_ACCEL_FILTER_WIDTH, LAUNCH_ACCEL_FILTER_FREQ);

    // Initialise burnout median filter
    initMedianFilter(&burnout_median_filter, BURNOUT_ACCEL_FILTER_WIDTH, BURNOUT_ACCEL_FILTER_FREQ);

    // Initialise apogee vertical velocity median filter
    initMedianFilter(&apogee_median_filter, VERTICAL_VELOCITY_FILTER_WIDTH, VERTICAL_VELOCITY_FILTER_FREQ);

    // Initialise apogee altitude exponential low pass filter
    initExpLowPassFilter(&altitude_exp_lp_filter, VERTICAL_VELOCITY_DETECT_FREQ, ALTITUDE_LP_FILTER_CUTOFF_FREQ, starting_altitude);
    last_altitude = starting_altitude;

    // Initialise apogee acceleration exponential low pass filter
    initExpLowPassFilter(&accel_low_pass_filter, VERTICAL_VELOCITY_DETECT_FREQ, ACCEL_LP_FILTER_CUTOFF_FREQ, 0.0);

    // Initialise landing detection exponential low pass filter
    initExpLowPassFilter(&landing_lp_filter, VERTICAL_VELOCITY_DETECT_FREQ, LANDING_VELOCITY_LP_CUTOFF_FREQ, -5.0);

    return 0;
}

bool detect_launch_accel(float ax, float ay, float az, float rocket_angle, uint32_t timestamp_ms)
{
    internal_state_fc.angle_from_vertical = rocket_angle;
    if (rocket_angle <= DEG_TO_RAD(PITCH_OVER_ANGLE_THRESHOLD) || M_PI - rocket_angle <= DEG_TO_RAD(PITCH_OVER_ANGLE_THRESHOLD))
    {
        float ax2 = ax * ax;
        float ay2 = ay * ay;
        float az2 = az * az;
        float acceleration_magnitude = sqrt(ax2 + ay2 + az2);

        updateMedianFilter(&launch_median_filter, acceleration_magnitude, (float)timestamp_ms / 1000.0);

        if (launch_median_filter.filledUp)
        {
            // Filter has been filled with valid values
            float filtered_acceleration = getMedianValue(launch_median_filter.values, (size_t)launch_median_filter.size);
            internal_state_fc.filtered_launch_detect_accel = filtered_acceleration;
            if (filtered_acceleration >= LAUNCH_ACCEL_THRESHOLD)
            {
                system_state.flight_state = LAUNCHED;
                system_state.launch_time = timestamp_ms;
                return true;
            }
        }
    }
    return false;
}

float accel_average[3] = {0};
uint32_t up_axis_loop_counter = 0;
bool calculate_up_axis(float ax, float ay, float az)
{
    if (up_axis_loop_counter < 20)
    {
        up_axis_loop_counter++;
        accel_average[0] += ax;
        accel_average[1] += ay;
        accel_average[2] += az;
        return false;
    }
    else
    {
        // Calculate averages
        accel_average[0] /= up_axis_loop_counter;
        accel_average[1] /= up_axis_loop_counter;
        accel_average[2] /= up_axis_loop_counter;

        // Determine the axis with largest absolute value
        float max_acceleration = fabs(accel_average[0]);
        int largest_index = 0;

        for (int i = 1; i < 3; i++)
        {
            if (fabs(accel_average[i]) > max_acceleration)
            {
                max_acceleration = fabs(accel_average[i]);
                largest_index = i;
            }
        }

        // Assign up axis
        if (largest_index == 0)
        {
            if (accel_average[largest_index] > 0)
            {
                system_state.up_axis = X_AXIS_POSITIVE;
            }
            else
            {
                system_state.up_axis = X_AXIS_NEGATIVE;
            }
        }
        else if (largest_index == 1)
        {
            if (accel_average[largest_index] > 0)
            {
                system_state.up_axis = Y_AXIS_POSITIVE;
            }
            else
            {
                system_state.up_axis = Y_AXIS_NEGATIVE;
            }
        }
        else
        {
            if (accel_average[largest_index] > 0)
            {
                system_state.up_axis = Z_AXIS_POSITIVE;
            }
            else
            {
                system_state.up_axis = Z_AXIS_NEGATIVE;
            }
        }

        // Reset state
        accel_average[0] = 0;
        accel_average[1] = 0;
        accel_average[2] = 0;
        up_axis_loop_counter = 0;
        return true;
    }
    return false;
}

bool detect_burnout_accel(float ax, float ay, float az, float altitude, uint32_t timestamp_ms)
{
    // Detect burnout with a timeout as back up
    float time_since_launch = (float)(timestamp_ms - system_state.launch_time) / 1000.0;
    if (time_since_launch >= MAX_MOTOR_BURN_TIME)
    {
        // Register burnout
        system_state.flight_state = BURNOUT;
    }
    float up_axis_reading = 0;
    // Pass the accelerometer reading into a filter that corresponds to the up axis
    switch (system_state.up_axis)
    {
    case X_AXIS_POSITIVE:
        up_axis_reading = ax;
        break;
    case X_AXIS_NEGATIVE:
        up_axis_reading = -ax;
        break;
    case Y_AXIS_POSITIVE:
        up_axis_reading = ay;
        break;
    case Y_AXIS_NEGATIVE:
        up_axis_reading = -ay;
        break;
    case Z_AXIS_POSITIVE:
        up_axis_reading = az;
        break;
    case Z_AXIS_NEGATIVE:
        up_axis_reading = -az;
        break;
    default:
        up_axis_reading = ax;
        break;
    }
    updateMedianFilter(&burnout_median_filter, up_axis_reading, (float)timestamp_ms / 1000.0);
    if (burnout_median_filter.filledUp)
    {
        // Filter has been filled with valid values
        float filtered_acceleration = getMedianValue(burnout_median_filter.values, (size_t)burnout_median_filter.size);
        internal_state_fc.filtered_burnout_detect_x_axis_accel = filtered_acceleration;
        if (filtered_acceleration <= BURNOUT_ACCEL_THRESHOLD)
        {
            // Register burnout
            system_state.flight_state = BURNOUT;
            system_state.burnout_time = timestamp_ms;
            system_state.burnout_altitude = altitude;
            return true;
        }
    }
    return false;
}

bool detect_apogee(float ax, float ay, float az, float altitude, uint32_t timestamp_ms)
{
    if ((float)(timestamp_ms - last_velocity_calculation_time) / 1000.0 >= (1.0 / VERTICAL_VELOCITY_DETECT_FREQ))
    {
        float dt = (timestamp_ms - last_velocity_calculation_time) / 1000.0;
        float filtered_altitude = updateExpLowPassFilter(&altitude_exp_lp_filter, altitude);
        float vertical_velocity = (filtered_altitude - last_altitude) / dt;
        updateMedianFilter(&apogee_median_filter, vertical_velocity, (float)timestamp_ms / 1000);
        float ax2 = ax * ax;
        float ay2 = ay * ay;
        float az2 = az * az;
        float acceleration_magnitude = sqrt(ax2 + ay2 + az2);
        float filtered_accel = updateExpLowPassFilter(&accel_low_pass_filter, acceleration_magnitude);
        internal_state_fc.filtered_apogee_detect_accel = filtered_accel;
        internal_state_fc.filtered_apogee_detect_altitude = filtered_altitude;
        if (apogee_median_filter.filledUp)
        {
            float filtered_vertical_velocity = getMedianValue(apogee_median_filter.values, apogee_median_filter.size);
            // Log internal state of system
            internal_state_fc.filtered_apogee_detect_vertical_velocity = filtered_vertical_velocity;

            if (filtered_vertical_velocity <= APOGEE_DETECT_VELOCITY_THRESHOLD)
            {
                if (filtered_accel <= APOGEE_DETECT_ACCEL_THRESHOLD)
                {
                    // Register apogee
                    system_state.flight_state = APOGEE;
                    system_state.drogue_deploy_time = timestamp_ms;
                    system_state.drogue_deploy_altitude = altitude;
                    return true;
                }
            }
        }
        last_velocity_calculation_time = timestamp_ms;
        last_altitude = filtered_altitude;
    }
    return false;
}

bool detect_main_deploy_altitude(float altitude, uint32_t timestamp_ms)
{
    float altitude_agl = altitude - system_state.starting_altitude;
    internal_state_fc.unfiltered_main_detect_agl_altitude = altitude_agl;
    if (altitude_agl <= MAIN_DEPLOY_ALTITUDE)
    {
        // Register main deploy altitude
        system_state.flight_state = MAIN_CHUTE_ALTITUDE;
        system_state.main_deploy_time = timestamp_ms;
        system_state.main_deploy_altitude = altitude;
        return true;
    }
    return false;
}

bool detect_landing(float altitude, uint32_t timestamp_ms)
{
    if ((float)(timestamp_ms - last_velocity_calculation_time) / 1000.0 >= (1.0 / VERTICAL_VELOCITY_DETECT_FREQ))
    {
        float dt = (timestamp_ms - last_velocity_calculation_time) / 1000.0;
        float vertical_velocity = (altitude - last_altitude) / dt;
        float filtered_vertical_velocity = updateExpLowPassFilter(&landing_lp_filter, vertical_velocity);
        internal_state_fc.filtered_landing_detect_vertical_velocity = filtered_vertical_velocity;
        if (fabs(filtered_vertical_velocity) <= LANDING_SPEED_THRESHOLD)
        {
            system_state.flight_state = LANDED;
            system_state.landing_time = timestamp_ms;
            system_state.landing_altitude = altitude;
            return true;
        }
        last_velocity_calculation_time = timestamp_ms;
        last_altitude = altitude;
    }
    return false;
}
