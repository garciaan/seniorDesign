#include "../adc/adc.h"

#define PSENSOR_MIN 0.4736328		//voltage at sea level (just below water)
#define PSENSOR_PIN	0		//PORTF pin that is attach to sensor

double get_depth_mpa();
double get_depth_feet();
