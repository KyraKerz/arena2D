#include "rectSpawn.h"

RectSpawn::RectSpawn() : _rectAreas(NULL), _vertexBuffer(0){
}

RectSpawn::~RectSpawn(){
	clear();
}

void RectSpawn::addRect(const zRect & r)
{
	_rects.push_back(r);
}

void RectSpawn::addCheeseRect(const zRect & main_rect, const std::vector<zRect> & holes)
{
	if(holes.size() == 0){
		addRect(main_rect);
		return;
	}
	float * x_anchors = new float[holes.size()*2 + 2];
	float * y_anchors = new float[holes.size()*2 + 2];
	int num_x_anchors = 2;
	x_anchors[0] = main_rect.x - main_rect.w;
	x_anchors[1] = main_rect.x + main_rect.w;
	int num_y_anchors = 2;
	y_anchors[0] = main_rect.y - main_rect.h;
	y_anchors[1] = main_rect.y + main_rect.h;

	std::vector<zRect> adj_holes(holes.size());
	int adj_holes_size = 0;
	int shape_count = 0;
	for(int i = 0; i < holes.size(); i++){
		zRect r= holes[i];
		if(!zRect::intersect(holes[i], main_rect, &r))
			continue;

		x_anchors[num_x_anchors] = r.x - r.w;
		num_x_anchors++;
		x_anchors[num_x_anchors] = r.x + r.w;
		num_x_anchors++;

		y_anchors[num_y_anchors] = r.y - r.h;
		num_y_anchors++;
		y_anchors[num_y_anchors] = r.y + r.h;
		num_y_anchors++;
		adj_holes[adj_holes_size] = r;
		adj_holes_size++;
	}

	// sort anchors
	f_selectionSort(x_anchors, num_x_anchors);
	f_selectionSort(y_anchors, num_y_anchors);

	float e = 0.0001f;
	// adding everything but the holes
	for(int i = 0; i < num_x_anchors-1; i++){
		for(int j = 0; j < num_y_anchors-1; j++){
			zRect r;
			r.w = (x_anchors[i+1] - x_anchors[i])/2.f;
			r.h = (y_anchors[j+1] - y_anchors[j])/2.f;
			if(r.w <= e || r.h <= e)
				continue;
			r.x = (x_anchors[i+1] + x_anchors[i])/2.f;
			r.y = (y_anchors[j+1] + y_anchors[j])/2.f;

			// check whether rect r corresponds to any hole rect
			bool correspondence = false;
			for(int hole_i = 0; hole_i < adj_holes_size; hole_i++){
				if(adj_holes[hole_i].contains(r, e)){
					correspondence = true;
					break;
				}
			}
			if(correspondence)// correspondence found -> do not add rect
				continue;
			addRect(r);
			shape_count++;
		}
	}

	delete[] x_anchors;
	delete[] y_anchors;
}

void RectSpawn::clear(){
	delete[](_rectAreas);
	_rectAreas = NULL;
	_rects.clear();
	if(_vertexBuffer)
		glDeleteBuffers(1, &_vertexBuffer);
	_vertexBuffer = 0;
}

void RectSpawn::calculateArea(){
	delete[](_rectAreas);
	int rects_size = _rects.size();
	_rectAreas = new float[rects_size];
	_areaSum = 0.f;
	for(int i = 0; i < rects_size; i++){
		float a = 2*_rects[i].w*_rects[i].h;
		_rectAreas[i] = a;
		_areaSum += a;
	}
}

void RectSpawn::render(){
	if(_vertexBuffer == 0){
		createVertexBuffer();
	}
	Z_SHADER->enableVertexArray(true);
	glBindBuffer(GL_ARRAY_BUFFER, _vertexBuffer);
	glVertexAttribPointer(Z_SHADER->getVertexLoc(), 2, GL_FLOAT, GL_FALSE, 0, 0);
	glDrawArrays(GL_TRIANGLES, 0, 12*_rects.size());
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	Z_SHADER->enableVertexArray(false);
}

void RectSpawn::createVertexBuffer(){
	glDeleteBuffers(1, &_vertexBuffer);
	int rect_size = _rects.size();
	if(rect_size == 0){// no rects to be rendered
		return;
	}
	glGenBuffers(1, &_vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, _vertexBuffer);
	float * verts = new float[rect_size*12];
	for(int i = 0; i < rect_size; i++){
		zRect r = _rects[i];
		verts[i*12 + 0 ] = r.x-r.w; verts[i*12 + 1 ] = r.y+r.h; 
		verts[i*12 + 2 ] = r.x-r.w; verts[i*12 + 3 ] = r.y-r.h; 
		verts[i*12 + 4 ] = r.x+r.w; verts[i*12 + 5 ] = r.y+r.h; 

		verts[i*12 + 6 ] = r.x+r.w; verts[i*12 + 7 ] = r.y+r.h; 
		verts[i*12 + 8 ] = r.x-r.w; verts[i*12 + 9 ] = r.y-r.h; 
		verts[i*12 + 10] = r.x+r.w; verts[i*12 + 11] = r.y-r.h; 
	}
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)*rect_size*12, verts, GL_STATIC_DRAW);
	delete[](verts);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void RectSpawn::getRandomPoint(zVector2D & v)
{
	if(_rects.size() == 0)
	{
		v.set(0,0);
		return;
	}
	// get random index, bigger rects are more likely
	int i = f_randomBuckets(_rectAreas, _rects.size(), &_areaSum);

	// get random point in rect
	zRect r = _rects[i];
	v.set(f_frandomRange(r.x-r.w, r.x+r.w), f_frandomRange(r.y-r.h, r.y+r.h));
}
