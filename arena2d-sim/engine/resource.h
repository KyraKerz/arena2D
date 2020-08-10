/*
 * Resource.h
 *
 *  Created on: May 19, 2018
 *      Author: zer0divider
 */

#ifndef RESOURCE_H_
#define RESOURCE_H_

#include <string>
#include "glew/glew.h"
#include <SDL2/SDL.h>

// sound library
#ifdef INCLUDE_AL
#include <AL/al.h>
#else
typedef int ALenum;
typedef size_t ALsizei;
#endif

#include "zFont.h"
#include "zVector2d.h"
#include "lodepng/lodepng.h"
#include <generated/resources/resource_list.h>
#include <assert.h>
#include "f_math.h"

#define RESOURCE_BATCH_SIZE 4 // how many threads will be run in parallel
extern volatile int RESOURCE_THREAD_STATUS[RESOURCE_BATCH_SIZE];// status of resource thread, 0: thread done, 1: thread running
extern SDL_mutex * RESOURCE_THREAD_STATUS_MUTEX;// mutual exclusion for RESOURCE_THREAD_STATUS
extern SDL_cond * RESOURCE_THREAD_FINISHED_SIGNAL;// signal send by thread

#define RESOURCE_THREAD_FINISH_SIGNALING(id) SDL_LockMutex(RESOURCE_THREAD_STATUS_MUTEX); RESOURCE_THREAD_STATUS[id] = 0; SDL_CondSignal(RESOURCE_THREAD_FINISHED_SIGNAL); SDL_UnlockMutex(RESOURCE_THREAD_STATUS_MUTEX); 

struct SpriteDescriptor
{
	SpriteDescriptor(){}
	SpriteDescriptor(int _tileW, int _tileH, int _sheetW, int _sheetH, int _margin){
		tileW = _tileW;	
		tileH = _tileH;
		sheetW = _sheetW;
		sheetH = _sheetH;
		margin = _margin;
	}
	//description:
	//tileW, tileH: dimension of a single frame (pixels)
	//sheetW, sheetH: dimensions of the whole sprite sheet (pixels)
	//margin: space (pixels) between the frames
	int tileW;
	int tileH;
	int sheetW;
	int sheetH;
	int margin;
};

// abstract resource
struct Resource
{
	Resource(RESOURCE_ID _id);
	static const char* getTypeName(int type){
		switch(type)
		{
			case RESOURCE_TYPE_IMAGE: return "Image";
			case RESOURCE_TYPE_SOUND: return "Sound";
			case RESOURCE_TYPE_FONT: return "Font";
			default: return "Unknown";
		}
	}
	virtual ~Resource(){}

	int getCount(){return _count;}
	int getType(){return _type;}
	RESOURCE_ID getID(){return _id;}
	bool isLoaded(){return _loaded;}

	struct sThreadData{
		sThreadData(){}
		sThreadData(RESOURCE_ID _id, const char * _path, int _offset): id(_id), path(_path), offset(_offset){}
		// parameters
		RESOURCE_ID id;
		Resource* r;//this pointer
		int thread_id;
		const char * path;
		int offset;

		// return
		unsigned char * data;
		union{
			struct {
				int w, h;
				GLenum format;
			}image;
			struct{
				ALsizei size;
				ALsizei samplerate;
				ALenum format;
			}sound;
			struct{
			}font;
		};
	};
	virtual int load(const sThreadData * data) = 0;
	virtual SDL_ThreadFunction getThreadFunction() = 0;
protected:
	RESOURCE_ID _id;
	int _count;
	int _type;
	bool _loaded;
};

// image resource
struct ResourceImage : public Resource
{
	ResourceImage(RESOURCE_ID id);
	~ResourceImage();

	int getTextureID(int id_offset = 0){assert(id_offset >= 0 && id_offset < _count); return _textureID[id_offset];}
	void bindTexture(int id_offset = 0){assert(id_offset >= 0 && id_offset < _count); glBindTexture(GL_TEXTURE_2D, _textureID[id_offset]);}
	int getWidth(){return _width;}
	int getHeight(){return _height;}
	// get scaling parameters for 2D transform (rendering image as texture) according to given target dimension
	void getScalingW(float target_w, zVector2D & dim){dim.set(target_w, target_w*(_height/(float)_width));}
	void getScalingH(float target_h, zVector2D & dim){dim.set(target_h*(_width/(float)_height), target_h);}

	static void optimizeBorderAlpha(Uint32 * pixels, int w, int h);
	void loadFromData(int w, int h, unsigned char * pixels, GLenum type, GLenum format, int id_offset = 0);
	void setTextureParameters(int id_offset = 0);

	static int loadThreaded(void * thread_data);// do work that can be parallelized
	int load(const sThreadData * data);// do work that can't be parallelized (set gl texture etc.)
	SDL_ThreadFunction getThreadFunction(){return &loadThreaded;}
private:
	GLuint *_textureID;
	int _width;
	int _height;
};

// font resource
struct ResourceFont : public Resource
{
	ResourceFont(RESOURCE_ID id);
	~ResourceFont();

	zFont* getFont(){return _font;}

	static int loadThreaded(void * thread_data){// return immediately for as multiple FT libraries would be neccessary for safe multithreading
		Resource::sThreadData * t = (Resource::sThreadData*)thread_data;
		RESOURCE_THREAD_FINISH_SIGNALING(t->thread_id);
		return 0;
	}// do work that can be parallelized
	int load(const sThreadData * data){return 0;}// do work that can't be parallelized
	SDL_ThreadFunction getThreadFunction(){return &loadThreaded;}
private:
	zFont * _font;
};


#endif /* RESOURCE_H_ */
