#if !defined(FILTER_H)
#define FILTER_H

#include <stdint.h>

typedef uint16_t adc_sample_t;

typedef struct Filter_bis {
    int size;
    int window_size;
    int last_id;
    float trimed_mean;
    float trimed_filtered_mean;
    uint16_t* samples;
    uint16_t* samples_sorted;
} Filter_bis;

void Filter_init(Filter_bis *filter, int size, int window_size, uint16_t *array1, uint16_t *array2, uint16_t initial_value);
uint16_t Filter_push(Filter_bis *filter, uint16_t value);

uint16_t Filter_getMedian(Filter_bis *filter);
float Filter_getTrimedMean(Filter_bis *filter);
float Filter_getTrimedFilteredMean(Filter_bis *filter);

void printSamplesSorted(Filter_bis *filter);
void printSamples(Filter_bis *filter);

#endif //FILTER_H