#ifndef LEVEL_EMPTY_H
#define LEVEL_EMPTY_H

#include "level.h"

class LevelEmpty : public Level
{
public:
	LevelEmpty(b2World * w, bool create_borders): Level(w, create_borders ? Level::EMPTY_BORDER : Level::EMPTY_NO_BORDER){
		if(create_borders){
			float level_half_size = _SETTINGS->stage.level_size/2.f;
			createBorder(b2Vec2(level_half_size, level_half_size));
		}
		init();
	}
	void reset(b2Vec2 & robot_position, bool hard_reset) override{
		randomGoalSpawnUntilValid(robot_position);
	}
private:
	void init(){
		float levelwidth = _SETTINGS->stage.level_size/2.f-_SETTINGS->stage.goal_size/2.f;
		float levelheight = _SETTINGS->stage.level_size/2.f-_SETTINGS->stage.goal_size/2.f;
		addRectToSpawnArea(zRect(0, 0, levelwidth, levelheight));
		calculateSpawnArea();
	}
};

#endif
