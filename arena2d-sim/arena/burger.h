#ifndef BURGER_H
#define BURGER_H

#include "physicsWorld.h"
#include "lidarCast.h"
#include <engine/zColor.h>
#include <engine/globalSettings.h>

// constants defining geometry and physical properties of the burger robot
#define BURGER_SAFE_RADIUS			0.12	// safe radius of circle from center of robot that encloses all robot parts (base and wheels)
#define BURGER_BASE_SIZE			0.130 	// length and width of burger base
#define BURGER_BASE_NON_BEVEL		0.082	// length of non-diagonal edge
#define BURGER_BASE_DENSITY			1.5		// 2D-density of base (has an effect on the total mass)
#define BURGER_WHEEL_OFFSET			0.034	// wheel offset along the axis the robot is facing
#define BURGER_WHEEL_DISTANCE		0.068	// distance of wheel from center of base
#define BURGER_WHEEL_RADIUS			0.032	// wheel radius
#define BURGER_WHEEL_WIDTH			0.018	// width of the wheels
#define BURGER_WHEEL_DENSITY		2 		// 2D-density of a wheel
#define BURGER_LIDAR_OFFSET			0		// lidar offset along the axis the robot is facing
#define BURGER_LIDAR_MAX_DISTANCE	3.5		// maximum distance in meters the lidar sensor can detect
#define BURGER_LIDAR_ACCURACY		0.0		// +- accuracy percentage
#define BURGER_LIDAR_NUM_SAMPLES	360		// total number of samples for 360 degrees
#define BURGER_DEFAULT_FORWARD_VELOCITY			0.15
#define BURGER_DEFAULT_ANGULAR_VELOCITY			0.75
#define BURGER_DEFAULT_STRONG_ANGULAR_VELOCITY	1.5

// visual constants
#define BURGER_LIDAR_POINT_COLOR	0xFF1020FF
#define BURGER_LIDAR_AREA_COLOR		0xFF102040
#define BURGER_LIDAR_POINT_SIZE		4
#define BURGER_TRAIL_BUFFER_SIZE	1000
#define BURGER_TRAIL_WIDTH			2
#define BURGER_TRAIL_COLOR			0x2255FFFF

struct Twist
{
	Twist(){}
	Twist(float _linear, float _angular): linear(_linear), angular(_angular){}
	float linear;	// linear velocity
	float angular; 	// angular velocity around center
};

class Burger
{
public:
	// discrete actions
	enum Action{FORWARD, FORWARD_LEFT, FORWARD_RIGHT, FORWARD_STRONG_LEFT, FORWARD_STRONG_RIGHT, BACKWARD, STOP, NUM_ACTIONS};

	Burger(b2World * world);
	~Burger();


	// color shader is assumed to be bound
	void renderScan(bool area);

	// color shader is assumed to be bound
	void renderTrail();
	void updateTrail();

	// reset burger position and velocities
	void reset(){reset(b2Vec2_zero);}
	void reset(const b2Vec2 & position);

	void resetTrail();

	// perform discrete action
	void performAction(Action a);

	// perform general action
	void performAction(const Twist & t);

	// called when settings have changed
	void updateLidar();

	// update lidar samples through ray casting
	void scan();
	const float* getSamples(int & num_samples){num_samples = _lidar->getNumSamples(); return _lidar->getDistances();}
	void getClosestObstacle(float & distance, float & angle);
	float getAngleFromSampleIndex(int i){return _lidar->getAngleFromIndex(i);}
	b2Body * getBody(){return _base;}
	b2Vec2 getPosition(){return _base->GetTransform().p;}
	const b2Fixture* getSafetyRadiusSensor(){return _safetyRadiusSensor;}

	// returns true on touch event
	bool beginContact(){_contactCount++; return (_contactCount == 1);}

	// returns true if all contacts have ended
	bool endContact(){bool ended = (_contactCount == 1); _contactCount--; if(_contactCount < 0)_contactCount=0; return ended;}

private:
	enum WheelIndicator{LEFT, RIGHT};
	b2Body * _base;
	b2FrictionJoint * _frictionJoint;
	b2MotorJoint * _motorJoint;
	b2Vec2 _wheelPosition[2];// relative position of the wheels center
	LidarCast * _lidar;
	b2Vec2 _closestPoint;
	GLuint _lidarBuffer;// gl buffer storing scanning data for rendering
	GLuint _trailBuffer;// gl buffer storing path robot has taken
	b2Vec2 _lastTrailPosition;
	int _trailVertexCount;
	int _lidarBufferCount; // how many samples are currently stored in buffer
	b2Fixture * _safetyRadiusSensor;
	int _contactCount;
};

#endif
