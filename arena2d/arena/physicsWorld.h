#ifndef PHYSICS_WORLD_H
#define PHYSICS_WORLD_H

#include <engine/renderer.h>
#include <engine/zSingleton.h>
#include <engine/zVector2d.h>
#include <engine/zVector3d.h>
#include <engine/zVector4d.h>
#include <engine/zMatrix4x4.h>
#include <box2d/box2d.h>
#include <engine/zRect.h>
#include <engine/zColor.h>
#include <list>
#define _PHYSICS PhysicsWorld::get()
#define _PHYSICS_WORLD _PHYSICS->getWorld()

//controlling debug rendering
#define PHYSICS_RENDER_COLLISIONS	0x00000001
#define PHYSICS_RENDER_SENSORS		0x00000002
#define PHYSICS_RENDER_DYNAMIC		0x00000004
#define PHYSICS_RENDER_STATIC		0x00000008
#define PHYSICS_RENDER_JOINTS		0x00000010
#define PHYSICS_RENDER_ALL			0xFFFFFFFF

//collision filter category bits (prefilter before solver)
#define COLLIDE_CATEGORY_BULLET				0x0002
#define COLLIDE_CATEGORY_COLLIDABLE_BULLET	0x0004
#define COLLIDE_CATEGORY_PLAYER				0x0008
#define COLLIDE_CATEGORY_STAGE				0x0010
#define COLLIDE_CATEGORY_BOX				0x0020
#define COLLIDE_CATEGORY_DONT_RENDER		0x0040
#define COLLIDE_CATEGORY_GOAL				0x0080

enum PhysicsDirection{RIGHT, UP, LEFT, DOWN};
extern const b2Vec2 PHYSICS_NEIGHBOUR_MAP[4];
extern const b2Vec2 PHYSICS_NEIGHBOUR_MAP_DIAGONAL[4];
//used for determining visible fixtures
class PhysicsWorldAABBQuery : public b2QueryCallback
{
public:
	PhysicsWorldAABBQuery(){}
	~PhysicsWorldAABBQuery(){}
	void reset(){_visibleFixtures.clear();}
	bool ReportFixture(b2Fixture * fixture)
	{
		//render sensors last
		if(fixture->IsSensor())
			_visibleFixtures.push_back(fixture);
		else
			_visibleFixtures.push_front(fixture);
		return true;
	}

std::list<b2Fixture*> _visibleFixtures;
};

//physics user data, all fixtures must contain a reference for identify box2d object
struct PhysicsUserData
{
	enum ObjectType {NONE, GAMING_OBJECT, BULLET, SOLID_STAGE};
	PhysicsUserData(){ptr = NULL; type = NONE;}
	PhysicsUserData(void * _ptr, ObjectType _type) : ptr(_ptr), type(_type){}

	void * ptr;
	ObjectType type;
};

class PhysicsWorld : public zTSingleton<PhysicsWorld>
{
public:
	PhysicsWorld();
	~PhysicsWorld();

	//creating box2d world
	//@param base_fps const frames per second the simulation runs on
	void init();

	//setting stretch factor for simulation steps (1.0 = normal)
	void setTimeStretch(float t){_timeStretch = t;}
	
	//setting simulation iteration
	void setIterations(int position_iterations, int velocity_iterations){_positionIterations = position_iterations; _velocityIterations = velocity_iterations;}
	void step(float time_step);

	void calculateVisibleFixtures(const zRect & visible_area){calculateVisibleFixturesWorld(_world, visible_area);}
	void calculateVisibleFixturesWorld(b2World * w, const zRect & visible_area);

	//getter 
	b2World* getWorld(){return _world;}
	b2Body * getStaticBody(){return _staticBody;}
	std::list<b2Fixture*>* getVisibleFixtures(){return &_aabbQuery._visibleFixtures;}

	//setting colors for collider
	void setDynamicColor(const zColor & c){_dynamicColor = c;}
	void setSensorColor(const zColor & c){_sensorColor = c;}
	void setSleepColor(const zColor & c){_sleepColor = c;}
	void setStaticColor(const zColor & c){_staticColor = c;}
	void setCollisionsColor(const zColor & c){_collisionsColor = c;}
	void setFillAlpha(float a){_fillAlpha = a;}

	//render colliders of every body 
	//NOTE: calculateVisibleFixtures() needs to be called before
	//@param flags: control what is rendered (see flags defined above) 
	void debugDraw(unsigned int flags, uint16 category_bits = 0xFFFF){debugDrawWorld(_world, flags, category_bits);}
	void debugDrawWorld(b2World * w, unsigned int flags, uint16 category_bits = 0xFFFF);

private:
	float _timeStretch;
	b2World * _world;
	int _positionIterations;
	int _velocityIterations;
	PhysicsWorldAABBQuery _aabbQuery;
	b2Body * _staticBody;//an empty static body that can be used as anchor joints
	//collider colors (for debug drawing)
	zColor _dynamicColor;
	zColor _sensorColor;
	zColor _sleepColor;
	zColor _staticColor;
	zColor _staticColor2;
	zColor _collisionsColor;
	zColor _kinematicColor;
	float _fillAlpha;
};

#endif
