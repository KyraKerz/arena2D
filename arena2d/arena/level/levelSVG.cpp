#include "levelSVG.h"
#include "engine/globalSettings.h"
void SpawnTesterCallback::test(b2World* w, SVGFile * f, float radius, float offset)
{
	b2Vec2 dim(f->getWidth(), f->getHeight());
	b2Vec2 start_pos(-dim.x/2, -dim.y/2);
	int x_iterations = (dim.x-radius*2)/offset + 1;
	int y_iterations = (dim.y-radius*2)/offset + 1;
	_currentTransform.q.Set(0);
	_testShape.m_p.Set(0,0);
	_testShape.m_radius = radius;
	for(int y = 0; y < y_iterations; y++){
		for(int x = 0; x < x_iterations; x++){
			b2AABB aabb;
			aabb.lowerBound.Set(start_pos.x + x*offset,
								start_pos.y + y*offset);
			aabb.upperBound.Set(aabb.lowerBound.x + radius*2,
								aabb.lowerBound.y + radius*2);
			_free = true;
			_currentTransform.p = aabb.GetCenter();
			w->QueryAABB((b2QueryCallback*)this, aabb);
			if(_free){
				f->addSpawnPosition(_currentTransform.p);
			}
		}
	}
}


bool SpawnTesterCallback::ReportFixture(b2Fixture* fixture)
{
	if(fixture->GetFilterData().categoryBits == COLLIDE_CATEGORY_STAGE){
		// test point in fixture
		if(b2TestOverlap(
			&_testShape, 0,
			fixture->GetShape(), 0,
			_currentTransform, fixture->GetBody()->GetTransform())){
			_free = false;
			return false;
		}
	}
	return true;
}

LevelSVG::LevelSVG(b2World * w) : Level(w, Level::SVG)
{
	_lastSpawnBufferIndex = -1;
	_currentFileIndex = -1;
	_spawnBuffer = 0;

	auto package_path = GLOBAL_PACKAGE_PATH.c_str();
	const char * delim = "/";
	const char * relative_folder_path = _SETTINGS->stage.svg_path;
    char folder_path[256];
	strncpy(folder_path,package_path,sizeof(folder_path));
	strncat(folder_path,delim,sizeof(folder_path));
	strncat(folder_path,relative_folder_path,sizeof(folder_path));



	INFO_F("Loading svg levels from \"%s\":", folder_path);
	DIR * d;
	d = opendir(folder_path);
	if(d == NULL){
		ERROR_F("Could not open folder \"%s\"!", folder_path);
		return;
	}

	struct dirent *dir;
	char path[256];
	int base_path_len = strlen(folder_path);
	strcpy(path, folder_path);
	// append '/'
	if(base_path_len > 0 && path[base_path_len-1] != '/')
	{
		path[base_path_len++] = '/';
	}
	
	// read every file in the directory and load it
	int error = 0;
	while((dir = readdir(d)) != NULL)
	{
		if(strcmp(dir->d_name, ".") && strcmp(dir->d_name, "..")){// ignore .. and . folders
			strcpy(path + base_path_len, dir->d_name);
			// check ending .svg
			int file_len = strlen(dir->d_name);
			if(file_len >= 4 &&
				(!strcmp(dir->d_name+file_len-4, ".svg") ||
				!strcmp(dir->d_name+file_len-4, ".SVG")))
			{
				INFO_F("  -> Loading \"%s\"...", path);
				SVGFile * f = new SVGFile(path);
				_files.push_back(f);
				if(f->load()){
					ERROR_F("Failed to load \"%s\"", path);
					error++;
				}
				else{
					// load last level and calculate spawn positions
					_currentFileIndex = _files.size()-1;
					loadFile(_currentFileIndex);
					SpawnTesterCallback tester;
					tester.test(w, f, BURGER_SAFE_RADIUS, BURGER_SAFE_RADIUS);
				}
			}
		}
	}
	closedir(d);
}

void LevelSVG::reset(b2Vec2 & robot_position, bool hard_reset)
{
	// no files loaded -> nothing to be done
	if(_currentFileIndex < 0){
		return;
	}

	// select random stage
	int new_stage = f_irandomRange(0, _files.size()-1);
	if(new_stage >= _files.size())
		new_stage = 0;
	if(new_stage != _currentFileIndex){
		loadFile(new_stage);
		_currentFileIndex = new_stage;
		hard_reset = true;
	}

	// select random spawn position until valid
	b2Vec2 goal_pos;
	do{
		goal_pos = _files[_currentFileIndex]->getRandomSpawn();
		if(hard_reset)
			robot_position = _files[_currentFileIndex]->getRandomSpawn();
	}while(!checkValidGoalSpawn(robot_position, goal_pos));

	// spawn goal
	spawnGoal(goal_pos);
}

void LevelSVG::loadFile(int index)
{
	// clear old level
	clearBodyList();

	// get stage
	SVGFile * f = _files[index];

	// create border
	b2Vec2 dim(f->getWidth(), f->getHeight());
	createBorder(0.5*dim);

	// create single Body containing all shapes
	b2BodyDef b;
	b.position.Set(-dim.x/2, dim.y/2);
	b.type = b2_staticBody;
	b2Body * static_body = _world->CreateBody(&b);
	_bodyList.push_back(static_body);

	// adding all shapes
	auto shapes = f->getShapes();
	for(int i = 0; i < shapes.size(); i++)
	{
		// creating fixture from shape and add to static body
		b2FixtureDef fix;
		fix.shape = shapes[i];
		fix.friction = 1;
		fix.restitution = 0;
		fix.filter.categoryBits = COLLIDE_CATEGORY_STAGE;
		static_body->CreateFixture(&fix);
	}
}

LevelSVG::~LevelSVG()
{
	for(int i = 0; i < _files.size(); i++)
		delete _files[i];
	
	glDeleteBuffers(1, &_spawnBuffer);
}

void LevelSVG::renderGoalSpawn()
{
	if(_currentFileIndex < 0)
		return;

	SVGFile * f = _files[_currentFileIndex];
	int pos_size = f->getSpawnPositionCount();
	if(_lastSpawnBufferIndex != _currentFileIndex)// create new buffer
	{
		glDeleteBuffers(1, &_spawnBuffer);
		glGenBuffers(1, &_spawnBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, _spawnBuffer);
		float * spawn_pos = new float[2*pos_size];
		for(int i = 0; i < pos_size; i++){
			b2Vec2 p = f->getSpawnPosition(i);
			spawn_pos[i*2+0] = p.x;
			spawn_pos[i*2+1] = p.y;
		}
		glBufferData(GL_ARRAY_BUFFER, sizeof(float)*2*pos_size, spawn_pos, GL_STATIC_DRAW);
		delete[] spawn_pos;
		_lastSpawnBufferIndex = _currentFileIndex;
	}
	else{
		glBindBuffer(GL_ARRAY_BUFFER, _spawnBuffer);
	}

	_RENDERER->resetModelviewMatrix();
	glPointSize(5);
	GLint vertex_loc = Z_SHADER->getVertexLoc();
	glEnableVertexAttribArray(vertex_loc);
	glVertexAttribPointer(vertex_loc, 2, GL_FLOAT, GL_FALSE, 0, 0);
	Z_SHADER->setColor(zColor(LEVEL_GOAL_SPAWN_COLOR));
	glDrawArrays(GL_POINTS, 0, pos_size);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDisableVertexAttribArray(vertex_loc);
}
