#include "filter.h"

#include <stdlib.h>
#include <string.h>

static int uint16_compare(const void *a, const void *b);

void Filter_init(Filter_bis *filter, int size, int window_size, adc_sample_t *array1, adc_sample_t *array2, adc_sample_t initial_value){
    filter->size = size;
    filter->window_size = window_size;
    filter->last_id = size - 1;
    filter->trimed_mean = initial_value;
    filter->trimed_filtered_mean = initial_value;
    filter->samples = array1;
    filter->samples_sorted = array2;

    for (int i = 0; i < size; i++) {
        array1[i] = initial_value;
        array2[i] = initial_value;
    }
}

adc_sample_t Filter_push(Filter_bis *filter, adc_sample_t value) {
    int size = filter->size;
    int window_size = filter->window_size;
    int current_id = (filter->last_id + 1)%size;
    adc_sample_t *samples = filter->samples;
    adc_sample_t *samples_sorted = filter->samples_sorted;
    adc_sample_t old_value = samples[current_id];

    filter->samples[current_id] = value;
    filter->last_id = current_id;

    // Search old value index in sorted array
    int old_id = 0;
    for (int i = size-1; i >= 0; i--) {
        if (samples_sorted[i] == old_value) {
            old_id = i;
            break;
        }
    }

    //Check if sorting and calculating the mean is avoidable
    if(value != old_value) {

        // Check if sorting is avoidable
        if(value <= samples_sorted[0]) {
            memmove(samples_sorted+1, samples_sorted, old_id*sizeof(adc_sample_t));
            samples_sorted[0] = value;
        }
        else if(value >= samples_sorted[size-1]) {
            memmove(samples_sorted+old_id, samples_sorted+old_id+1, (size-old_id-1)*sizeof(adc_sample_t));
            samples_sorted[size-1] = value;
        }
        else {
            int old_id = 0;
            for(int i = size - 1; i >= 0; i--) { // Checking from the end because it's more likely to get high values
                if(samples_sorted[i] == old_value) {
                    old_id = i;
                    break;
                }
            }

            int left_id = 0;
            for(int i = size - 1; i >= 0; i--) {
                if(samples_sorted[i] <= value) {
                    left_id = i;
                    break;
                }
            }

            if(old_id <= left_id) {
                memmove(samples_sorted+old_id, samples_sorted+old_id+1, (left_id-old_id)*sizeof(adc_sample_t));
                samples_sorted[left_id] = value;
            }
            else if(old_id > left_id) {
                memmove(samples_sorted+left_id+1, samples_sorted+left_id, (old_id-left_id)*sizeof(adc_sample_t));
                samples_sorted[left_id+1] = value;
            }
        }

        // Calculate the mean
        if(window_size >= size)
        {
            uint32_t sum = 0;
            for(int i = 0; i < size; i++)
            {
                sum += samples_sorted[i];
            }
            filter->trimed_mean = sum/(float)window_size;
        }
        else if(window_size <= 1)
        {
            filter->trimed_mean = samples_sorted[size/2];
        }
        else
        {
            int end = size/2 + window_size/2;
            int start = end - window_size + 1;
            adc_sample_t sum = 0;
            for(int i = start; i <= end; i++)
            {
                sum += samples_sorted[i];
            }
            filter->trimed_mean = sum/(float)window_size;
        }
    }

    filter->trimed_filtered_mean = 0.6f*filter->trimed_filtered_mean + 0.4f*filter->trimed_mean;

    return old_value;
}

uint16_t Filter_getMedian(Filter_bis *filter) {
    return filter->samples_sorted[filter->size / 2];
}

float Filter_getTrimedMean(Filter_bis *filter) {
    return filter->trimed_mean;
}

float Filter_getTrimedFilteredMean(Filter_bis *filter) {
    return filter->trimed_filtered_mean;
}

int uint16_compare(const void *a, const void *b) {
    uint16_t val_a = *(const uint16_t*)a;
    uint16_t val_b = *(const uint16_t*)b;
    return (val_a > val_b) - (val_a < val_b);
}
