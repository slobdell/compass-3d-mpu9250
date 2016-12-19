#include "compass3d.h"
#include <iostream>

// These values are obtained by calibrating the magnetometer.  The easiest way to do this
// is to log x, y, and z values over time and try to orient the magnetometer in all possible
// directions.  The offset in each axis should be such that the min and max values for each axis
// with the offset applied should average to 0.
float xOffset = -43.6189453125;
float yOffset = 13.7592773437;
float zOffset = -9.2390625;

// magnetic declination varies by region. Can be found at http://www.magnetic-declination.com/
// current declination is based off of San Francisco
float magneticDeclinationOffset = 13.8;

int main() {
	Compass3D *compass = new Compass3D();
	while(1) {
		compass->updateFromSensor(xOffset, yOffset, zOffset);
		float pitch = compass->getPitch();
		float yaw = compass->getYaw(magneticDeclinationOffset);
		float roll = compass->getRoll();
		printf("%.2f, %.2f, %.2f\n", pitch, yaw, roll);
	}
	return 0;
}
