#include "filter.h"
#include "math.h"

#define MAX_DOUBLE_MEAN_TRIES 3

void FilterInit(Filter *filter, uint16_t *array, int size)
{
    filter->size = size;
    filter->samples = array;
    filter->sum = 0;
    filter->mean = 0;
    filter->wrap_index = 0;
    for (int i = 0; i < size; i++) {
        filter->samples[i] = 0;
    }
}

void FilterSetAll(Filter *filter, uint16_t value)
{
    for (int i = 0; i < filter->size; i++) {
        filter->samples[i] = value;
    }
    filter->mean = value;
    filter->sum = value * filter->size;
}

uint16_t FilterGetOldest(Filter *filter)
{
    return filter->samples[filter->wrap_index];
}

uint16_t FilterGetNewest(Filter *filter)
{
    int index = (filter->size + filter->wrap_index - 1) % filter->size;
    return filter->samples[index];
}

uint16_t FilterPush(Filter *filter, uint16_t sample)
{
    uint16_t oldest = filter->samples[filter->wrap_index];
    filter->sum -= oldest;
    filter->samples[filter->wrap_index] = sample;
    filter->sum += sample;
    filter->mean = filter->sum / (float)filter->size;
    filter->wrap_index = (filter->wrap_index + 1) % filter->size;
    return oldest;
}

uint16_t FilterMean(Filter *filter)
{
    return filter->sum / filter->size;
}

float FilterMeanFloat(Filter *filter)
{
    return filter->mean;
}

//exclude values out of the standard deviation multiplied by a factor 
uint16_t FilterDoubledMean(Filter *filter, float factor)
{
    float deviation = FilterStandardDeviation(filter);
    int validated_values = 0;
    uint32_t sum = 0;
    uint16_t doubled_mean = 0;

    // we want at least 1/3 of the values to be validated
    safe_while(validated_values < (filter->size / 3 + 1), MAX_DOUBLE_MEAN_TRIES)
    {
        validated_values = 0;
        uint16_t min = filter->mean - factor * deviation;
        uint16_t max = filter->mean + factor * deviation;
        sum = 0;
        for (int i = 0; i < filter->size; i++)
        {
            if (filter->samples[i] >= min && filter->samples[i] <= max)
            {
                sum += filter->samples[i];
                validated_values++;
            }
        }
        factor *= 1.5f;
    }

    if( validated_values > (filter->size / 3 + 1) )
        doubled_mean = sum / validated_values;
    else
        doubled_mean = filter->mean;

    return doubled_mean;
}

//exclude values out of the standard deviation multiplied by a factor 
float FilterDoubledMeanFloat(Filter *filter, float factor)
{
    float deviation = FilterStandardDeviation(filter);
    int validated_values = 0;
    double sum = 0.0f;
    float doubled_mean = 0.0f;

    // we want at least 1/3 of the values to be validated
    safe_while(validated_values < (filter->size / 3 + 1), MAX_DOUBLE_MEAN_TRIES)
    {
        validated_values = 0;
        float min = filter->mean - factor * deviation;
        float max = filter->mean + factor * deviation;
        sum = 0.0f;
        for (int i = 0; i < filter->size; i++)
        {
            if (filter->samples[i] >= min && filter->samples[i] <= max)
            {
                sum += filter->samples[i];
                validated_values++;
            }
        }
        factor *= 1.5f;
    }

    if( validated_values > (filter->size / 3 + 1) )
        doubled_mean = sum / validated_values;
    else
        doubled_mean = filter->mean;

    return doubled_mean;
}

float FilterStandardDeviation(Filter *filter)
{
    double sum = 0.0f;
    for (int i = 0; i < filter->size; i++) {
        float diff = filter->samples[i] - filter->mean;
        diff *= diff;
        sum += diff;
    }
    return sqrtf( sum / (filter->size - 1) );
}

