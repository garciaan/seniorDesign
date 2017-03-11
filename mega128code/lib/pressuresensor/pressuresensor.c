
#include "pressuresensor.h"

double get_depth_mpa(){
	return 1.2 * (get_voltage(PSENSOR_PIN) - PSENSOR_MIN)/(4.5-PSENSOR_MIN);
}
double get_depth_feet(){
	return 334.56229215 * get_depth_mpa();
}

