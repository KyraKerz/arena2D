#ifndef LEVELSTATIC_H
#define LEVELSTATIC_H

#include "level.h"
#include "wanderer.h"


// spawn area
#define LEVELSTATIC_SIDE_PANEL_SPAWN_W 0.3
#define LEVELSTATIC_SIDE_PANEL_SPAWN_H 0.3

class LevelStatic : public Level
{
public:
	LevelStatic(b2World * w, bool dynamic = false) : Level(w, dynamic ? Level::STATIC_DYNAMIC : Level::STATIC), _dynamic(dynamic){
	}

	~LevelStatic(){
		freeWanderers();
	}

	void reset(b2Vec2 & robot_position, bool hard_reset) override
	{
		// clear old bodies and spawn area
		clearBodyList();
		clearSpawnArea();
		if(_dynamic)
			freeWanderers();

		float half_width = _SETTINGS->stage.level_size/2.f;
		float half_height = _SETTINGS->stage.level_size/2.f;
		createBorder(b2Vec2(half_width, half_height));

		int num_obstacles = _SETTINGS->stage.num_obstacles;

		std::vector<zRect> robot_hole(1);
		std::vector<zRect> holes(num_obstacles);
		float half_goal_size = _SETTINGS->stage.goal_size/2.f;
		for(int i = 0; i < num_obstacles; i ++){
			RectSpawn spawn;
			b2Body * b = generateRandomBody(false, _SETTINGS->stage.min_obstacle_size/2, _SETTINGS->stage.max_obstacle_size/2, &holes[i]);
			robot_hole[0].set(robot_position.x, robot_position.y,
								BURGER_SAFE_RADIUS+holes[i].w,
								BURGER_SAFE_RADIUS+holes[i].h);
			// avoid obstacles spawning directly on robot
			spawn.addCheeseRect(zRect(0, 0, half_width-holes[i].w, half_height-holes[i].h), robot_hole);
			spawn.calculateArea();
			zVector2D p;
			spawn.getRandomPoint(p);
			holes[i].x += p.x;
			holes[i].y += p.y;
			holes[i].w += half_goal_size;
			holes[i].h += half_goal_size;
			b->SetTransform(b2Vec2(p.x, p.y), 0);
			_bodyList.push_back(b);
		}
		// spawning dynamic obstacles
		if(_dynamic){
			float obstacle_radius = _SETTINGS->stage.dynamic_obstacle_size/2.f;
			float obstacle_speed = _SETTINGS->stage.obstacle_speed;
			RectSpawn spawn;
			std::vector<zRect> dyn_holes(num_obstacles+1);
			for(int i =0; i < num_obstacles; i++){
				dyn_holes[i] = holes[i];
				dyn_holes[i].w += -half_goal_size + obstacle_radius;
				dyn_holes[i].h += -half_goal_size + obstacle_radius;
			}
			dyn_holes[num_obstacles] = zRect(robot_position.x, robot_position.y, BURGER_SAFE_RADIUS+obstacle_radius, BURGER_SAFE_RADIUS+obstacle_radius);
			spawn.addCheeseRect(zRect(0, 0, half_width-obstacle_radius, half_height-obstacle_radius), dyn_holes);
			spawn.calculateArea();
			for(int i = 0; i < _SETTINGS->stage.num_dynamic_obstacles; i++){
				zVector2D p;
				spawn.getRandomPoint(p);
				Wanderer * w = new Wanderer(_world, obstacle_radius, b2Vec2(p.x, p.y), obstacle_speed, 0.1, 0.0, 0);
				_wanderers.push_back(w);
			}
		}
		// adding spawn area
		zRect main_rect(0,0, half_width-half_goal_size, half_height-half_goal_size);
		addCheeseRectToSpawnArea(main_rect, holes);
		calculateSpawnArea();


		randomGoalSpawnUntilValid(robot_position);
	}

	b2Body* generateRandomBody(bool dynamic, float min_radius, float max_radius, zRect * aabb){
		b2BodyDef def;
		def.type = dynamic ? b2_dynamicBody : b2_staticBody;
		def.allowSleep = false;
		def.linearDamping = 0;
		def.angularDamping = 0;
		int vert_count = f_irandomRange(3, 6);
		b2PolygonShape shape;
		shape.m_count = vert_count;
		zVector2D verts[8];
		float radius_x = f_frandomRange(min_radius, max_radius); 
		float radius_y = f_frandomRange(min_radius, max_radius); 
		b2Vec2 max_v(-10000, -10000);
		b2Vec2 min_v(10000, 10000);
		float rotation = f_frandomRange(0, 2*M_PI);
		for(int i = 0; i < vert_count; i++){
			float angle = M_PI*2*(i/static_cast<float>(vert_count));
			verts[i].set(cos(angle)*radius_x, sin(angle)*radius_y);
			verts[i].rotate(rotation);
			if(verts[i].x > max_v.x){
				max_v.x = verts[i].x;
			}
			if(verts[i].y > max_v.y){
				max_v.y = verts[i].y;
			}
			if(verts[i].x < min_v.x){
				min_v.x = verts[i].x;
			}
			if(verts[i].y < min_v.y){
				min_v.y = verts[i].y;
			}
		}
		if(aabb != NULL){
			aabb->x = (max_v.x+min_v.x)/2.0f;
			aabb->y = (max_v.y+min_v.y)/2.0f;
			aabb->w = (max_v.x-min_v.x)/2.0f;
			aabb->h = (max_v.y-min_v.y)/2.0f;
		}
		shape.Set((b2Vec2*)verts, vert_count);
		b2FixtureDef fix;
		fix.shape = &shape;
		fix.filter.categoryBits = COLLIDE_CATEGORY_STAGE;
		fix.friction = 1;
		fix.restitution = 0;
		fix.density = 1;
		b2Body * b = _world->CreateBody(&def);
		b->CreateFixture(&fix);
		return b;
	}

	void update(const b2Transform & robot_transform) override{
		for(std::list<Wanderer*>::iterator it = _wanderers.begin(); it != _wanderers.end(); it++){
			(*it)->update();
		}
	}

	void freeWanderers(){
		for(std::list<Wanderer*>::iterator it = _wanderers.begin(); it != _wanderers.end(); it++){
			delete (*it);
		}
		_wanderers.clear();
	}
private:
	bool _dynamic;// if set to true, create dynamic obstacles in addition to static
	std::list<Wanderer*> _wanderers;
};

#endif
