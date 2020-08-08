#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "zer0/zVector2d.h"

struct Triangle
{
	Triangle(){}
	Triangle(zVector2D _v0, zVector2D _v1, zVector2D _v2){v0 = _v0; v1 = _v1; v2 = _v2;}
	void set(zVector2D _v0, zVector2D _v1, zVector2D _v2){v0 = _v0; v1 = _v1; v2 = _v2;}
	zVector2D v0, v1, v2;
};

#endif