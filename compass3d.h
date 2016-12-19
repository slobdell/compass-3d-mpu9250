#include <cmath>
#include "InertialSensor.h"
#include "MPU9250.h"
#include <sys/time.h>

#define G_SI 9.80665
#define PI 3.14159265359


class Compass3D {
	private:
		float pitch;
		float yaw;
		float roll;
		float ax, ay, az, gx, gy, gz, mx, my, mz;
		InertialSensor *inertialSensor;
		float deltaSeconds;
		unsigned long previousTime;
		float q[4];

		void madgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
		
	public:
		Compass3D();
		void updateFromSensor(float magXOffset, float magYOffset, float magZOffset);
		float getPitch();
		float getYaw(float magneticDeclinationOffset);
		float getRoll();
};
