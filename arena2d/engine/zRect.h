#ifndef Z_RECT_H
#define Z_RECT_H

#include "zVector2d.h"
#include "f_math.h"

struct zRect
{
	zRect(float r_x, float r_y, float r_w, float r_h): x(r_x), y(r_y), w(r_w), h(r_h){}
	zRect(const zVector2D & pos, const zVector2D & dim): x(pos.x), y(pos.y), w(dim.x), h(dim.y){}
	zRect(){}
	void set(float r_x, float r_y, float r_w, float r_h){x = r_x; y = r_y; w = r_w; h = r_h;}
	void set(const zVector2D & pos, const zVector2D & dim){x=pos.x; y=pos.y; w=dim.x; h=dim.y;}
	void setPos(const zVector2D & p){x = p.x; y = p.y;}
	void setDim(const zVector2D & d){w = d.x; h = d.y;}
	void toFulldim(){x -= w; y -= h; w*=2; h*=2;}
	void toHalfdim(){w/=2; h/=2;x += w; y += h; }//convert from upper left point center to middle center with half dimensions

	// check whether a given point lies within the rect
	bool checkPoint(const zVector2D & point, bool assume_half_dim = true){
		if(assume_half_dim)
			return point.x <= x+w && point.x >= x-w && point.y <= y+h && point.y >= y-h;
		else
			return point.x <= x+w && point.x >= x && point.y <= y+h && point.y >= y;
	}

	static bool intersect(const zRect & a, const zRect & b, zRect * intersection_rect = NULL){
		if(a.x+a.w > b.x-b.w && a.x-a.w < b.x+b.w &&
		   a.y+a.h > b.y-b.h && a.y-a.h < b.y+b.h){
			if(intersection_rect != NULL){
				float left = f_fmax(a.x-a.w, b.x-b.w);
				float right = f_fmin(a.x+a.w, b.x+b.w);
				float down = f_fmax(a.y-a.h, b.y-b.h);
				float up = f_fmin(a.y+a.h, b.y+b.h);
				intersection_rect->set((right+left)/2.f, (up+down)/2.f, (right-left)/2.f, (up-down)/2.f);
			}
			return true;
		}
		else{
			return false;
		}
	}

	// returns true if given rect lies inside this rect fully
	bool contains(const zRect & r, float epsilon = 0.f)const{
		return r.x+r.w <= x+w+epsilon && r.x-r.w >= x-w-epsilon &&
				r.y+r.h <= y+h+epsilon && r.y-r.h >= y-h-epsilon;
	}

	float x;
	float y;
	float w;//mostly used as half-dimensions but needs to be specified by application
	float h;
};

#endif
