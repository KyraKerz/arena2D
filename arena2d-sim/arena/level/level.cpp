#include "level.h"
Level::Level(b2World * w, Level::Type t): _goal(NULL), _type(t), _world(w)
{
}

Level::~Level()
{
	clearBodyList();
	// free goal
	if(_goal != NULL)
		_world->DestroyBody(_goal);
}

void Level::clearBodyList()
{
	// free all bodies that belong to this level
	for(std::list<b2Body*>::iterator it = _bodyList.begin(); it != _bodyList.end(); it++) {
		_world->DestroyBody(*it);
	}
	_bodyList.clear();
}

void Level::spawnGoal(const b2Vec2 & pos)
{
	if(_goal != NULL){
		_world->DestroyBody(_goal);
	}
	b2BodyDef def;
	def.type = b2_staticBody;
	def.position = pos;
	_goal = _world->CreateBody(&def);
	b2CircleShape c;
	c.m_p.Set(0, 0);
	c.m_radius = _SETTINGS->stage.goal_size/2.f;
	b2FixtureDef fix;
	fix.isSensor = true;
	fix.filter.categoryBits = COLLIDE_CATEGORY_GOAL;
	fix.shape = &c;
	_goal->CreateFixture(&fix);
}

void Level::createBorder(const b2Vec2 & dim)
{
	b2BodyDef b;
	b.type = b2_staticBody;
	b2Body * body = _world->CreateBody(&b);

	b2Vec2 v[4];
	v[0].Set(-dim.x, dim.y);
	v[1].Set(dim.x, dim.y);
	v[2].Set(dim.x, -dim.y);
	v[3].Set(-dim.x, -dim.y);
	b2ChainShape chain;
	chain.CreateLoop(v, 4);
	b2FixtureDef f;
	f.shape = &chain;
	f.friction = 1;
	f.restitution = 0;
	f.filter.categoryBits = COLLIDE_CATEGORY_STAGE;
	body->CreateFixture(&f);
	_bodyList.push_back(body);
}

void Level::addCircle(const b2Vec2 & pos, float radius)
{
	b2BodyDef b;
	b.type = b2_staticBody;
	b2Body * body = _world->CreateBody(&b);

	b2CircleShape circle;
	circle.m_p = pos;
	circle.m_radius = radius;
	b2FixtureDef f;
	f.shape = &circle;
	f.friction = 1;
	f.restitution = 0;
	f.filter.categoryBits = COLLIDE_CATEGORY_STAGE;
	body->CreateFixture(&f);
	_bodyList.push_back(body);
}

void Level::renderGoalSpawn()
{
	Z_SHADER->setColor(zColor(LEVEL_GOAL_SPAWN_COLOR));
	_goalSpawnArea.render();
}

b2Vec2 Level::getRandomPoint(float x_min, float x_max, float y_min, float y_max)
{
	return b2Vec2(f_frandomRange(x_min, x_max), f_frandomRange(y_min, y_max));
}

b2Vec2 Level::getRandomPointCircle(const b2Vec2 & center, float radius)
{
	float r = radius *( sqrt(f_random()));
	float theta = f_random() * 2 * M_PI;
	return b2Vec2(r*cos(theta), r*sin(theta)) + center;
}
