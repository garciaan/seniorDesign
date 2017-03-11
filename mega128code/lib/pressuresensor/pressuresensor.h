#include "../adc/adc.h"

#define PSENSOR_MIN .5		//voltage at sea level (just above water)
#define PSENSOR_PIN	0		//PORTF pin that is attach to sensor

double get_depth_mpa();
double get_depth_feet();
