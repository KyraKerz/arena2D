#ifndef RECT_SPAWN_AREA_H
#define RECT_SPAWN_AREA_H

#include <engine/zRect.h>
#include <engine/renderer.h>
#include <engine/f_math.h>
#include <vector>
#include <engine/zVector2d.h>


class RectSpawn{
public:
	RectSpawn();
	~RectSpawn();

	// adding rect with half dimensions
	void addRect(const zRect & r);
	void clear();
	void addCheeseRect(const zRect & main_rect, const std::vector<zRect> & holes);

	// call when done adding rects
	void calculateArea();

	// debug rendering rects vertex attribute must be enabled, color has to be set by callee
	void render();

	// create vertex buffer from current rects
	void createVertexBuffer();

	void getRandomPoint(zVector2D & v);

private:
	GLuint _vertexBuffer;
	std::vector<zRect> _rects;
	float * _rectAreas;
	float _areaSum;
};

#endif
