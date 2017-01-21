#include "compass3d.h"

const float SENSOR_TO_COMPASS_OFFSET = 180.0;
const float TO_RADIANS_COEFF = PI / 180.0;
const float TO_DEGREES_COEFF = 1.0 / TO_RADIANS_COEFF;
const float GYRO_MEASUREMENT_ERROR = 40.0 * TO_RADIANS_COEFF;
const float beta = sqrt(3.0 / 4.0) * GYRO_MEASUREMENT_ERROR;


struct timeval tv;
unsigned long getTimestampUs() {
    gettimeofday(&tv,NULL);
    return 1000000 * tv.tv_sec + tv.tv_usec;
}


float normalizeHeading(float angle) {
    while(angle < 0) {
        angle += 360.0;
    }
    while(angle >= 360.0) {
        angle -= 360.0;
    }
    return angle;
}


void Compass3D::madgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltaSeconds;
    q2 += qDot2 * deltaSeconds;
    q3 += qDot3 * deltaSeconds;
    q4 += qDot4 * deltaSeconds;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}


void Compass3D:: mutateRelativeAcceleration(float a12, float a22, float a31, float a32, float a33, float pitch) {
    float gravity[3];
    gravity[0] = a32;
    gravity[1] = a31;
    gravity[2] = a33;

    float pureForwardAcceleration = -1 * (ay - gravity[1]);
    float pureUpAcceleration = -1 * (az - gravity[2]);
    float pureRightAcceleration = -1 * (ax + gravity[0]);

    float groundForwardAcceleration = (
        cos(pitch * TO_RADIANS_COEFF) * pureForwardAcceleration -
        sin(pitch * TO_RADIANS_COEFF) * pureUpAcceleration
    );
    float groundUpwardAcceleration = (
        cos(pitch * TO_RADIANS_COEFF) * pureUpAcceleration + 
        sin(pitch * TO_RADIANS_COEFF) * pureForwardAcceleration
    ); // SBL pitch yaw and roll rotations aren't commutative, so just leaving out roll entirely
	forwardAccelerationValues->enqueue(groundForwardAcceleration);
	upwardAccelerationValues->enqueue(groundUpwardAcceleration);
}

void Compass3D::mutateQuaternionFromSensor(float magXOffset, float magYOffset, float magZOffset) {
    unsigned long currentTime = getTimestampUs();
    deltaSeconds = (currentTime - previousTime) / 1000000.0;
    previousTime = currentTime;
    inertialSensor->update();

    inertialSensor->read_accelerometer(&ax, &ay, &az);
    inertialSensor->read_gyroscope(&gx, &gy, &gz);
    inertialSensor->read_magnetometer(&mx, &my, &mz);

	/*
	float gMag = sqrt(ax * ax + ay * ay + az * az);
	DEBUG_COUNTER2++;
	gTotal += gMag;
	printf("Avg g: %f\n", gTotal / DEBUG_COUNTER2);
	*/


    ax /= G_SI;
    ay /= G_SI;
    az /= G_SI;

    madgwickQuaternionUpdate(
        -ax,
        ay,
        az,
        gx,
        -gy,
        -gz,
        my + magYOffset,
        -(mx + magXOffset),
        mz + magZOffset
    );
}

void Compass3D::memsetPitchYawRoll(float pitch, float yaw, float roll) {
	pitchValues->enqueue(pitch);
	float lastYaw = yawValues -> getMostRecentItem(0);
	float negYaw = yaw;
	float posYaw = yaw;
	while(1) {
		if(abs(posYaw - lastYaw) <= 180.0) {
			yawValues -> enqueue(posYaw);
			break;
		}
		if (abs(negYaw - lastYaw) <= 180.0) {
			yawValues -> enqueue(negYaw);
			break;
		}
		negYaw -= 360.0;
		posYaw += 360.0;
	}
	rollValues->enqueue(roll);
}

void Compass3D::mutateAbsoluteAcceleration(float a12, float a22, float a31, float a32, float a33, float magneticDeclinationOffset) {
    float gravity[3];
    gravity[0] = a32;
    gravity[1] = a31;
    gravity[2] = a33;

	// I'm computing absolute acceleration differently here from elsewhere because I need to
	// maintain consistency with the quaternions...this is arguably bad with the difference in
	// the other section of this code
    float pureAx = (ax + gravity[0]); // this might be incorrect...
    float pureAy = (ay - gravity[1]);
    float pureAz = (az - gravity[2]);

	// this effectively converts a quaternion to a matrix and then applies that rotation to a
	// vector.  Normally I wouldn't do these whacky comments and just move to an aptly named
	// function, but we care about performance a bit 
	float num1 = q[0] * 2.0;
	float num2 = q[1] * 2.0;
	float num3 = q[2] * 2.0;
	float num4 = q[0] * num1;
	float num5 = q[1] * num2;
	float num6 = q[2] * num3;
	float num7 = q[0] * num2;
	float num8 = q[0] * num3;
	float num9 = q[1] * num3;
	float num10 = q[3] * num1;
	float num11 = q[3] * num2;
	float num12 = q[3] * num3;

	float accelerationEast = (1.0 - (num5 + num6)) * pureAx + (num7 - num12) * pureAy + (num8 + num11) * pureAz;
	float accelerationNorth = (num7 + num12) * pureAx + (1.0 - (num4 + num6)) * pureAy + (num9 - num10) * pureAz;
	float accelerationUp = -1 * ((num8 - num11) * pureAx + (num9 + num10) * pureAy + (1.0 - (num4 + num5)) * pureAz);

    float sinMagneticDeclination = sin(magneticDeclinationOffset * TO_RADIANS_COEFF);
	float easternNorthComponent = sinMagneticDeclination * accelerationEast;
	float northernEasternComponent = -sinMagneticDeclination * accelerationNorth;

	accelerationNorth += northernEasternComponent;
	accelerationEast += easternNorthComponent;


	absoluteAccelerationMutex.lock();
	easternAccelerationValues->enqueue(accelerationEast);
	northernAccelerationValues->enqueue(accelerationNorth);
	absoluteUpwardAccelerationValues->enqueue(accelerationUp);
	absoluteAccelerationMutex.unlock();
}

void Compass3D::startPopAbsoluteAcceleration(float magXOffset, float magYOffset, float magZOffset, float magneticDeclinationOffset) {
	// because I'm calling these methods from Go, it's not trivial to return a struct...therefore
	// adding a setup and teardown method that will make 3 independent method calls atomic.
	absoluteAccelerationMutex.lock();
	if(easternAccelerationValues->length() == 0){
		// we need at least 1 value to successfully pop
		absoluteAccelerationMutex.unlock();
		updateFromSensor(magXOffset, magYOffset, magZOffset, magneticDeclinationOffset);
		absoluteAccelerationMutex.lock();
	}
}

void Compass3D::finishPopAbsoluteAcceleration(){
	absoluteAccelerationMutex.unlock();
}

float Compass3D::popAbsoluteAccelerationNorth(){
	float value = northernAccelerationValues->average();
	northernAccelerationValues->empty();
	return value;
}

float Compass3D::popAbsoluteAccelerationEast(){
	float value = easternAccelerationValues->average();
	easternAccelerationValues->empty();
	return value;
}

float Compass3D::popAbsoluteAccelerationUp(){
	float value = absoluteUpwardAccelerationValues->average();
	absoluteUpwardAccelerationValues->empty();
	return value;
}

void Compass3D::updateFromSensor(float magXOffset, float magYOffset, float magZOffset, float magneticDeclinationOffset) {
	mutateQuaternionFromSensor(magXOffset, magYOffset, magZOffset);

    float a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    float a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    float a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    float a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
    float a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

    pitch = asinf(a32) * TO_DEGREES_COEFF;
    yaw = SENSOR_TO_COMPASS_OFFSET + (atan2f(a12, a22) * TO_DEGREES_COEFF);
    roll = -atan2f(a31, a33) * TO_DEGREES_COEFF;
    transformForAircraft();

	memsetPitchYawRoll(pitch, yaw, roll);
	mutateRelativeAcceleration(a12, a22, a31, a32, a33, pitch);
	mutateAbsoluteAcceleration(a12, a22, a31, a32, a33, magneticDeclinationOffset);
}

void Compass3D::transformForAircraft() {
    // FIXME this is code that's making assumptions about how we want to orient the device.
    float originalPitch = pitch;
    float originalYaw = yaw;
    float originalRoll = roll;

    pitch = originalRoll * -1;
    roll = originalPitch;
    yaw = originalYaw + 90.0;
}

float Compass3D::getPitch() {
	return pitchValues->average();
}

float Compass3D::getYaw(float magneticDeclinationOffset) {
    return normalizeHeading(
        yawValues->average() + magneticDeclinationOffset
    );
}

float Compass3D::getRoll() {
	return rollValues->average();
}
float Compass3D::getGroundForwardAcceleration(){
	return forwardAccelerationValues->average();
}
float Compass3D::getGroundUpwardAcceleration(){
	return upwardAccelerationValues->average();
}

Compass3D::Compass3D(int expectedReadFrequency) {
	int evictingQueueMemory = SENSOR_READ_FREQUENCY / expectedReadFrequency;

	pitchValues = new EvictingQueue(evictingQueueMemory);
	yawValues = new EvictingQueue(evictingQueueMemory);
	rollValues = new EvictingQueue(evictingQueueMemory);

	forwardAccelerationValues = new EvictingQueue(evictingQueueMemory);
	upwardAccelerationValues = new EvictingQueue(evictingQueueMemory);

	// buffer sizes here are conservatively higher since we explicitly pop the value for this case
	// because we want to integrate against these
	easternAccelerationValues = new EvictingQueue(2 * evictingQueueMemory);
	northernAccelerationValues = new EvictingQueue(2 * evictingQueueMemory);
	absoluteUpwardAccelerationValues = new EvictingQueue(2 * evictingQueueMemory);

    inertialSensor = new MPU9250();
    inertialSensor->initialize();

    pitch = 0;
    roll = 0;
    yaw = 0;
    deltaSeconds = 0;
    previousTime = 0;
    q[0] = 1.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;
}
