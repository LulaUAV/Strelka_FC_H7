/*
 * digital_filter.h
 *
 *  Created on: Feb 8, 2024
 *      Author: Angus McLennan
 */

#ifndef INC_DIGITAL_FILTER_H_
#define INC_DIGITAL_FILTER_H_

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#define MAX_MEDIAN_FILTER_SIZE 10			// Used to define the max median filter size. Beware, large value results in excessive computation due to O(n^2) sorting algorithm

typedef struct {
    float values[MAX_MEDIAN_FILTER_SIZE];
    int size;
    int currentIndex;
    bool filledUp;
    float updateFrequency; // In Hz
    float lastUpdateTime;
} MedianFilter_t;

typedef struct {
    float updateFrequency;	// Hz
    float cutoffFrequency;	// Hz
    float prevOutput; // Previous output state
    bool initialized; // Flag to indicate if the filter has been initialized
    float alpha;
} ExpLowPassFilter_t;

/*
 * Function to initialise a median filter
 * Inputs:
 * MedianFilter_t *filter: Median filter object
 * int size: Number of elements in the median filter
 * float updateFrequency: Frequency at which the filter is updated in Hz
 * Returns: void
 */
void initMedianFilter(MedianFilter_t *filter, int size, float updateFrequency);

/*
 * Function to update the median filter
 * Inputs:
 * MedianFilter_t *filter: Median filter object
 * float newValue: New value entered into the filter
 * float updateTime_s: Current timestamp when function is called in seconds
 * Returns: void
 */
void updateMedianFilter(MedianFilter_t *filter, float newValue, float updateTime_s);

/*
 * Function to calculate the median value of an array
 * Inputs:
 * float *array: Array of float values
 * size_t size: Size of input array
 * Returns: Median value of input array
 */
float getMedianValue(float *array, size_t size);

/*
 * Function to initialise a low pass filter object
 * Inputs:
 * ExpLowPassFilter_t *filter: Low pass filter object
 * float updateFrequency: Low pass filter update frequency
 * float cutoffFrequency: Low pass filter cutoff frequency
 * float initialInput: Initial value of the low pass filter
 */
void initExpLowPassFilter(ExpLowPassFilter_t *filter, float updateFrequency, float cutoffFrequency, float initialInput);

/*
 * Function that updates the low pass filter
 * Inputs:
 * ExpLowPassFilter_t *filter: Low pass filter object
 * float input: New value passed into the low pass filter
 * Return: Low pass filter update value
 */
float updateExpLowPassFilter(ExpLowPassFilter_t *filter, float input);

#endif /* INC_DIGITAL_FILTER_H_ */
