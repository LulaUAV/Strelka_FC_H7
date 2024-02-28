/*
 * digital_filter.c
 *
 *  Created on: Feb 8, 2024
 *      Author: Angus McLennan
 */

#include "digital_filter.h"

void initMedianFilter(MedianFilter_t *filter, int size, float updateFrequency)
{
    filter->size = size > MAX_MEDIAN_FILTER_SIZE ? MAX_MEDIAN_FILTER_SIZE : size;
    filter->currentIndex = 0;
    filter->filledUp = false;
    filter->updateFrequency = updateFrequency;
    filter->lastUpdateTime = 0;
    filter->lastUpdateTime = 0.0;
}

void updateMedianFilter(MedianFilter_t *filter, float newValue, float updateTime_s)
{
    if (updateTime_s - filter->lastUpdateTime >= 1 / filter->updateFrequency)
    {
        // Sufficient time has elapsed since last update
        filter->lastUpdateTime = updateTime_s;
        filter->values[filter->currentIndex] = newValue;
        filter->currentIndex++;
        if (filter->currentIndex >= filter->size)
        {
            filter->currentIndex = 0;
            filter->filledUp = true;
        }
    }
}

float getMedianValue(float *array, size_t size)
{
    // Sort the array (bubble sort for simplicity)
    for (int i = 0; i < size - 1; i++)
    {
        for (int j = 0; j < size - i - 1; j++)
        {
            if (array[j] > array[j + 1])
            {
                // Swap
                float temp = array[j];
                array[j] = array[j + 1];
                array[j + 1] = temp;
            }
        }
    }

    // Calculate median
    int middleIndex = size / 2;
    if (size % 2 == 0)
    {
        // If even number of elements, take the average of the two middle values
        return (array[middleIndex - 1] + array[middleIndex]) / 2.0;
    }
    else
    {
        // If odd number of elements, take the middle value
        return array[middleIndex];
    }
}

// Initialize the exponential low-pass filter
void initExpLowPassFilter(ExpLowPassFilter_t *filter, float updateFrequency, float cutoffFrequency, float initialInput)
{
    filter->updateFrequency = updateFrequency;
    filter->cutoffFrequency = cutoffFrequency;
    filter->prevOutput = initialInput;
    filter->alpha = 1.0 - exp(-2.0 * M_PI * filter->cutoffFrequency / filter->updateFrequency);
}

// Update the exponential low-pass filter with a new input value
float updateExpLowPassFilter(ExpLowPassFilter_t *filter, float input)
{
    // Calculate the new output using the exponential low-pass filter formula
    float output = filter->alpha * input + (1 - filter->alpha) * filter->prevOutput;

    // Update the previous output state
    filter->prevOutput = output;

    return output;
}
