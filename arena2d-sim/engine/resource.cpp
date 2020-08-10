/*
 * Resources.cpp
 *
 *  Created on: May 19, 2018
 *      Author: zer0divider
 */

#include "resource.h"
volatile int RESOURCE_THREAD_STATUS[RESOURCE_BATCH_SIZE];
SDL_mutex * RESOURCE_THREAD_STATUS_MUTEX;// mutual exclusion for RESOURCE_THREAD_STATUS
SDL_cond * RESOURCE_THREAD_FINISHED_SIGNAL;

Resource::Resource(RESOURCE_ID id)
{
	_id = id;
	const ResourceDescriptor * r = &RESOURCES_DESCRIPTOR[(int)id];
	_type =	r->type; 
	_loaded = false;
	if(r->start_count >= 0 && r->end_count >= 0)
		_count = r->end_count - r->start_count + 1;
	else
		_count = 1;
}

//FONT
ResourceFont::ResourceFont(RESOURCE_ID id) : Resource(id)
{
	// TODO: implement image collection
	if(_count > 1){
		ERROR("Multiple Fonts for one resource not implemented yet!");
		return;
	}

	_font = new zFont();
	const ResourceDescriptor * desc = &RESOURCES_DESCRIPTOR[(int)_id];
	if(_font->loadFromFile(desc->path) < 0){// loading failed
		delete _font;
		_font = NULL;
	}
	else{
		_loaded = true;
	}
}

ResourceFont::~ResourceFont()
{
	if(_font != NULL)
		delete _font;
}

