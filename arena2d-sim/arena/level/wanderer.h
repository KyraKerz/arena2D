#ifndef WANDERER_H
#define WANDERER_H
#include "../physicsWorld.h"
#include <engine/zVector2d.h>

class Wanderer{
public:
	Wanderer(b2World * w, float radius, const b2Vec2 & position, float velocity, float change_rate, float stop_rate, int id = 0);
	~Wanderer();
	void update();

	void reset(const b2Vec2 & position);

	b2Vec2 getPosition(){return _body->GetTransform().p;}
	int getID(){return _id;}

	b2Body* getBody(){return _body;}
	float getRadius(){return _radius;}
private:
	void updateVelocity();
	b2Body * _body;
	float _velocity; // constant velocity
	float _changeRate;// [0, 1] how often to change velocity
	float _stopRate; // [0, 1] how likely velocity is set to 0 on change
	float _radius;
	int _id;
};

#endif
