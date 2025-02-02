#if !defined(FILTER_H)
#define FILTER_H

#include "common.h"
#include "stdint.h"

typedef struct Filter
{
    int size;
    uint16_t *samples;
    uint32_t sum;
    float mean;
    int wrap_index; // index of the oldest sample, instead of shifting all the samples we wrap around the array
} Filter;

void FilterInit(Filter *filter, uint16_t *array, int size);

void FilterSetAll(Filter *filter, uint16_t value);

uint16_t FilterGetOldest(Filter *filter);

uint16_t FilterGetNewest(Filter *filter);

uint16_t FilterPush(Filter *filter, uint16_t sample);

uint16_t FilterMean(Filter *filter);

float FilterMeanFloat(Filter *filter);

//exclude values to far from the mean in the range allowed
uint16_t FilterDoubledMean(Filter *filter, float factor);

//exclude values to far from the mean in the range allowed
float FilterDoubledMeanFloat(Filter *filter, float factor);

float FilterStandardDeviation(Filter *filter);

#endif // FILTER_H
