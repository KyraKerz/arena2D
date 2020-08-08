#ifndef SVG_FILE_H
#define SVG_FILE_H

#include <vector>
#include <arena/physicsWorld.h>
#include <arena/rectSpawn.h>
#include <engine/f_math.h>
#include <engine/globalSettings.h>
#include <engine/zVector2d.h>
#include <engine/zStringTools.h>

#define SVG_SCALE (1/1000.0)

struct SVGAttribute{
	SVGAttribute(){name = NULL; value = NULL; next = NULL;}
	SVGAttribute(const char *_name, int name_len, const char * _value, int value_len){
		name = new char[name_len+1];
		value = new char[value_len+1];
		memcpy(name, _name, name_len);
		memcpy(value, _value, value_len);
		name[name_len] = '\0';
		value[value_len] = '\0';
		next = NULL;
	}
	~SVGAttribute(){delete[]name; delete[]value; delete next;}
	char * name;
	char * value;
	SVGAttribute * next;
};

struct SVGTag{
	SVGTag(const char * _name, int name_len){
		name = new char[name_len+1];
		memcpy(name, _name, name_len);
		name[name_len] = '\0';
		attribs = NULL;
		children = NULL;
		next = NULL;
		value = NULL;
	}
	void setValue(const char * _value, int value_len){
		delete[] value;
		value = new char[value_len+1];
		memcpy(value, _value, value_len);
		value[value_len] = '\0';
	}
	~SVGTag(){
		delete[]name;
		delete attribs;
		delete[] value;
		delete next;
	}
	// returns value of given attribute or NULL if attribute does not exist
	const char* getAttribute(const char * name){
		for(SVGAttribute * a = attribs; a != NULL; a = a->next){
			if(!strcmp(a->name, name))
				return a->value;
		}
		return NULL;
	}
	// search for immediate children, returns NULL if not found
	SVGTag* getChild(const char * name){
		for(SVGTag * c = children; c != NULL; c = c->next){
			if(!strcmp(c->name, name))
				return c;
		}
		return NULL;
	}

	void print(int indents = 0);

	static void printIndents(int indents){
		for(int i = 0; i < indents; i++){
			printf("    ");
		}
	}
	char * name;
	SVGAttribute * attribs;
	char * value;
	SVGTag * children;
	SVGTag * next;
};

struct SVGTransform{
	SVGTransform(const char * t);
	SVGTransform(const b2Mat33 &m): matrix(m){}
	void transform(b2Vec2 & v, float z);
	b2Mat33 matrix;
};

class SVGFile
{
public:
	SVGFile(const char * path){
		_path = new char[strlen(path)+1];
		strcpy(_path, path);
		_width = 0;
		_height = 0;
	}
	~SVGFile(){
		delete[]_path;
		for(int i = 0; i < _shapes.size(); i++)
			delete _shapes[i];
	}

	// load from file
	// returns 0 on success, 1 on error
	int load();

	// returns root
	static SVGTag* parseXML(const char * text);
	void freeXML(SVGTag * root){delete root;}

	float getWidth()const{return _width;}
	float getHeight()const{return _height;}
	const std::vector<b2Shape*>& getShapes(){return _shapes;}
	const char* getPath(){return _path;}
	void addSpawnPosition(const b2Vec2 & pos){_spawnPositions.push_back(pos);}
	b2Vec2 getRandomSpawn(){return _spawnPositions[f_irandomRange(0, _spawnPositions.size()-1)];}
	int getSpawnPositionCount(){return _spawnPositions.size();}
	b2Vec2 getSpawnPosition(int index){return _spawnPositions[index];}
private:
	enum XMLError{END_OF_FILE, EXPECTED_TAG, EXPECTED_EQUALS, EXPECTED_VALUE, VALUE_LINEBREAK, END_TAG_NEQ_START_TAG, EXPECTED_OPEN_BRACKET, EXPECTED_CLOSE_BRACKET};
	static void throwParseError(XMLError e, int line);

	std::vector<b2Shape*> _shapes;
	float _width, _height;
	char * _path;
	// SpawnArea
	std::vector<b2Vec2> _spawnPositions;
};

#endif
