#ifndef _COMPASS_3D_H
#define _COMPASS_3D_H

#include <cmath>
#include <mutex>
#include "InertialSensor.h"
#include "evictingQueue.h"
#include "MPU9250.h"
#include <sys/time.h>

//#define G_SI 9.80665
// sensor is off...get this value by reading raw magnitutde of ax, ay, and az
#define G_SI 9.572791
#define PI 3.14159265359
#define SENSOR_READ_FREQUENCY 1600

unsigned long getTimestampUs();

class Compass3D {
	private:
		std::mutex absoluteAccelerationMutex;
		float pitch;
		float yaw;
		float roll;
		float ax, ay, az, gx, gy, gz;

		EvictingQueue *pitchValues;
		EvictingQueue *yawValues;
		EvictingQueue *rollValues;

		EvictingQueue *forwardAccelerationValues;
		EvictingQueue *upwardAccelerationValues;

		EvictingQueue *easternAccelerationValues;
		EvictingQueue *northernAccelerationValues;
		EvictingQueue *absoluteUpwardAccelerationValues;

		InertialSensor *inertialSensor;
		float deltaSeconds;
		unsigned long previousTime;
		float q[4];

		void madgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
		void transformForAircraft();
		void mutateQuaternionFromSensor(float magXOffset, float magYOffset, float magZOffset);
		void mutateRelativeAcceleration(float a12, float a22, float a31, float a32, float a33, float pitch);
		void memsetPitchYawRoll(float pitch, float yaw, float roll);
		void mutateAbsoluteAcceleration(float a12, float a22, float a31, float a32, float a33, float magneticDeclinationOffset);
		
	public:

		float mx, my, mz;
		Compass3D(int expectedReadFrequency);
		void updateFromSensor(float magXOffset, float magYOffset, float magZOffset, float magneticDeclinationOffset);
		float getPitch();
		float getYaw(float magneticDeclinationOffset);
		float getRoll();

		float getGroundForwardAcceleration();
		float getGroundUpwardAcceleration();

		void startPopAbsoluteAcceleration(float magXOffset, float magYOffset, float magZOffset, float magneticDeclinationOffset);
		void finishPopAbsoluteAcceleration();

		float popAbsoluteAccelerationNorth();
		float popAbsoluteAccelerationEast();
		float popAbsoluteAccelerationUp();
};
#endif
