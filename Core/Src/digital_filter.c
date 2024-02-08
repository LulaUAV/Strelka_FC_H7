/*
 * digital_filter.c
 *
 *  Created on: Feb 8, 2024
 *      Author: Angus McLennan
 */

#include "digital_filter.h"


void initMedianFilter(MedianFilter_t *filter, int size, float updateFrequency) {
    filter->size = size > MAX_FILTER_SIZE ? MAX_FILTER_SIZE : size;
    filter->currentIndex = 0;
    filter->filledUp = false;
    filter->updateFrequency = updateFrequency;
    filter->currentUpdateCount = 0;
    filter->lastUpdateTime = 0.0;
}


void updateMedianFilter(MedianFilter *filter, float newValue, float updateTime_s) {
	if(updateTime_s - filter->lastUpdateTime >= 1/filter->updateFrequency) {
		// Sufficient time has elapsed since last update
		filter->lastUpateTime = updateTime_s;
	    filter->values[filter->currentIndex] = newValue;
	    filter->currentIndex++;
	    if (filter->currentIndex >= filter->size) {
	        filter->currentIndex = 0;
	        filter->filledUp = true;
	    }
	}
}

float getMedianValue(float *array, size_t size) {
    // Sort the array (bubble sort for simplicity)
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - i - 1; j++) {
            if (array[j] > array[j + 1]) {
                // Swap
                float temp = array[j];
                array[j] = array[j + 1];
                array[j + 1] = temp;
            }
        }
    }

    // Calculate median
    int middleIndex = size / 2;
    if (size % 2 == 0) {
        // If even number of elements, take the average of the two middle values
        return (array[middleIndex - 1] + array[middleIndex]) / 2.0;
    } else {
        // If odd number of elements, take the middle value
        return array[middleIndex];
    }
}
