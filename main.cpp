#include "compass3d.h"
#include <iostream>

#define ACTUAL_GRAVITY 9.80665
#define WARMUP_SECONDS 10
#define EXPECTED_READ_FREQUENCY 16
#define STANDARD_DEVIATION_SAMPLES EXPECTED_READ_FREQUENCY * 60


float xOffset = -43.6189453125;
float yOffset = 13.7592773437;
float zOffset = -9.2390625;

float magneticDeclinationOffset = 13.8;

int counter = 0;
unsigned long previousTime = -1;


float computeAccelerometerVariance(float *values, int numSamples) {
	// in the context of using the variance for a kalman filter, the mean SHOULD be 0 even if that's
	// not what the sensor produces...in this way we can automatically offset the bias the sensor
	// gives
	const float mean = 0.0;
	float total = 0.0;
	for(int i=0; i<numSamples;i++) {
		float value = values[i];
		total += (value - mean) * (value- mean);
	}
	return total / numSamples;
}

int main() {
	int i = 0;
	float *northSamples = new float[STANDARD_DEVIATION_SAMPLES];
	float *eastSamples = new float[STANDARD_DEVIATION_SAMPLES];
	float *upSamples= new float[STANDARD_DEVIATION_SAMPLES];

    float vNorth = 0;
    float vEast = 0;
    float vUp = 0;
    Compass3D *compass = new Compass3D(EXPECTED_READ_FREQUENCY);
    while(1) {

        compass->updateFromSensor(xOffset, yOffset, zOffset, magneticDeclinationOffset);
        if(counter % 100 == 0){
            float pitch = compass->getPitch();
            float yaw = compass->getYaw(magneticDeclinationOffset);
            float roll = compass->getRoll();
            float forward = compass->getGroundForwardAcceleration();
            float up = compass->getGroundUpwardAcceleration();

            compass->startPopAbsoluteAcceleration(xOffset, yOffset, zOffset, magneticDeclinationOffset);
            float averageNorthAcc = ACTUAL_GRAVITY * compass->popAbsoluteAccelerationNorth();
            float averageEastAcc = ACTUAL_GRAVITY * compass->popAbsoluteAccelerationEast();
            float averageUpAcc = ACTUAL_GRAVITY * compass->popAbsoluteAccelerationUp();
            compass->finishPopAbsoluteAcceleration();
            unsigned long currentTime = getTimestampUs();
            double deltaSeconds = (currentTime - previousTime) / 1000000.0;
            if(previousTime == -1) {
                // don't integrate velocity on very first read
                previousTime = currentTime;
                continue;
            }
            previousTime = currentTime;
			if(counter >= SENSOR_READ_FREQUENCY * WARMUP_SECONDS) {
				if (i == STANDARD_DEVIATION_SAMPLES) {
					i = 0;
					float varN = computeAccelerometerVariance(northSamples, STANDARD_DEVIATION_SAMPLES);
					float varE = computeAccelerometerVariance(eastSamples, STANDARD_DEVIATION_SAMPLES);
					float varUp = computeAccelerometerVariance(upSamples, STANDARD_DEVIATION_SAMPLES);
					printf("Stanard deviation, N: %f, E: %f, Up: %f\n", varN, varE, varUp);
				}
				vNorth += averageNorthAcc * deltaSeconds;
				vEast += averageEastAcc * deltaSeconds;
				vUp += averageUpAcc * deltaSeconds;
				northSamples[i] = averageNorthAcc;
				eastSamples[i] = averageEastAcc;
				upSamples[i] = averageUpAcc;
				i++;

				//printf("North: %.1f, East: %.1f, Up: %.1f\n", vNorth, vEast, vUp);
			}
            // printf("North: %.1f, East: %.1f, Up: %.1f\n", averageNorthAcc, averageEastAcc, averageUpAcc);
        }
        counter++;
    }
    return 0;
}
