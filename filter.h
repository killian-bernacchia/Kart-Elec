#if !defined(FILTER_H)
#define FILTER_H

#include <stdint.h>

typedef uint16_t adc_sample_t;

typedef struct Filter {
    int size;
    int window_size;
    int last_id;
    float trimed_mean;
    float trimed_filtered_mean;
    uint16_t* samples;
    uint16_t* samples_sorted;
} Filter;

void Filter_init(Filter *filter, int size, int window_size, uint16_t *array1, uint16_t *array2, uint16_t initial_value);
uint16_t Filter_push(Filter *filter, uint16_t value);

uint16_t Filter_getMedian(Filter *filter);
float Filter_getTrimedMean(Filter *filter);
float Filter_getTrimedFilteredMean(Filter *filter);

void printSamplesSorted(Filter *filter);
void printSamples(Filter *filter);

#endif //FILTER_H