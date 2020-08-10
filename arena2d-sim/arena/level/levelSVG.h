#ifndef LEVEL_SVG_H
#define LEVEL_SVG_H 

#include "level.h"
#include <dirent.h>
#include <engine/zStringTools.h>
#include "SVGFile.h"


class SpawnTesterCallback : public b2QueryCallback
{
public:
	void test(b2World * w, SVGFile * f, float radius, float offset);

	bool ReportFixture(b2Fixture* fixture) override;

private:
	bool _free;
	b2CircleShape _testShape;
	b2Transform _currentTransform;
};

class LevelSVG : public Level
{
public:
	LevelSVG(b2World* w);
	~LevelSVG();

	void reset(b2Vec2 & robot_position, bool hard_reset) override;

	void renderGoalSpawn()override;
private:
	void loadFile(int index);
	std::vector<SVGFile*> _files;
	int _currentFileIndex;

	GLuint _spawnBuffer;
	int _lastSpawnBufferIndex;
};

#endif
