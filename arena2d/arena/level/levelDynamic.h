#ifndef LEVEL_DYNAMIC_H
#define LEVEL_DYNAMIC_H

#include "level.h"
#include "wanderer.h"

// level with moving obstacles
class LevelDynamic : public Level
{
public:
	LevelDynamic(b2World * w) : Level(w, Level::DYNAMIC){
		float half_width = _SETTINGS->stage.level_size/2.f;
		float half_height = _SETTINGS->stage.level_size/2.f;
		createBorder(b2Vec2(half_width, half_height));
		float obstacle_radius = _SETTINGS->stage.dynamic_obstacle_size/2.f;

		addRectToSpawnArea(zRect(0, 0, half_width-obstacle_radius, half_height-obstacle_radius));
		calculateSpawnArea();

		// creating moving obstacles
		for(int i = 0; i < 4; i++){
			_wanderers.push_back(new Wanderer(w, obstacle_radius, b2Vec2(0,0),
									_SETTINGS->stage.obstacle_speed, 0.1f, 0.0f, 0));
		}
	}

	~LevelDynamic(){
		freeWanderers();
	}

	void reset(b2Vec2 & robot_position, bool hard_reset) override{
		randomGoalSpawnUntilValid(robot_position);
		if(hard_reset){
			float x_offset = _SETTINGS->stage.level_size/4.0f;
			float y_offset = _SETTINGS->stage.level_size/4.0f;
			int count = 0;
			for(std::list<Wanderer*>::iterator it = _wanderers.begin(); it != _wanderers.end(); it++){
				b2Vec2 r = PHYSICS_NEIGHBOUR_MAP_DIAGONAL[count];
				(*it)->reset(b2Vec2(r.x*x_offset,r.y*y_offset));
				count++;
			}
		}
	}


	void update(const b2Transform & robot_transform){
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
	std::list<Wanderer*> _wanderers;
};

#endif
