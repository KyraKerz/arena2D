/*
 * ResourceHandler.cpp
 *
 *  Created on: May 19, 2018
 *      Author: zer0divider
 */

#include "resourceHandler.h"


ResourceHandler::ResourceHandler()
{
	memset(_resources, 0, sizeof(Resource*)*RESOURCES_NUMBER);
	_resourcesLoaded = 0;

	// creating mutex and cond-signal
	RESOURCE_THREAD_STATUS_MUTEX = SDL_CreateMutex();
	RESOURCE_THREAD_FINISHED_SIGNAL = SDL_CreateCond();

	// generating file paths
	for(int i = 0; i < RESOURCES_NUMBER; i++){
		const ResourceDescriptor * desc = &RESOURCES_DESCRIPTOR[i];
		int count = 1;
		if(desc->end_count >= 0 && desc->start_count >= 0)
			count = desc->end_count - desc->start_count + 1;
		char ** load_paths = new char*[count];
		int len = strlen(desc->path);
		if(count > 1){
			// searching for $-signs
			const char * p = desc->path;
			int sign_start = -1;
			int sign_end = -1;
			int p_i = 0;
			while(1) {
				if(p[p_i] == '$' && sign_start < 0){
					sign_start = p_i;
				}
				else if(p[p_i] != '$' && sign_start >= 0){
					sign_end = p_i;
					break;
				}
				if(p[p_i] == '\0')
					break;
				p_i++;
			}
			// check if start/end has been found
			if(sign_start < 0 || sign_end < 0){
				WARNING_F("Multiple images demanded but no '$' have been found in in path: %s", desc->path);
				for(int j = 0; j < count; j++){
					load_paths[j] = new char[len+1];
					strcpy(load_paths[j], desc->path); 
				}
			}
			else{
				int num_signs = sign_end-sign_start;
				// creating load-paths
				for(int j = 0; j < count; j++){
					load_paths[j] = new char[len+1];
					load_paths[j][0] = '\0';
					memcpy(load_paths[j], desc->path, sign_start);
					sprintf(load_paths[j]+sign_start, "%.*d%s", num_signs, desc->start_count+j, desc->path+sign_end);
				}
			}
		}
		else{
			load_paths[0] = new char[len+1];
			strcpy(load_paths[0], desc->path);
		}
		_loadPaths[i].set((RESOURCE_ID)i, load_paths, count);
	}

}

ResourceHandler::~ResourceHandler()
{
	// free mutex and cond signal
	SDL_DestroyMutex(RESOURCE_THREAD_STATUS_MUTEX);
	SDL_DestroyCond(RESOURCE_THREAD_FINISHED_SIGNAL);
	freeAll();
}

void ResourceHandler::freeAll()
{
	INFO("Freeing all resources:");
	if(_resourcesLoaded > 0)
	{
		for(int i = 0; i < RESOURCES_NUMBER; i++) {
			freeResource((RESOURCE_ID)i);
		}
	}
}

int ResourceHandler::loadList(const RESOURCE_ID * list, int size, LoadCallback* callback, uint32_t flags)
{
	INFO("Loading resources:");
	Uint32 t = SDL_GetTicks();
	int error_count = 0;
	if(callback != NULL)
		callback->updatePercentage(0.f);

	std::list<Resource::sThreadData*> t_data;
	// getting all thread data in a single linear list
	for(int i = 0; i < size; i++){
		if(isLoaded(list[i]))continue;// resource already loaded
		LoadPath *l = &_loadPaths[(int)list[i]];
		for(int j = 0; j < l->count; j++){
			t_data.push_back(new Resource::sThreadData(l->id, l->paths[j], j));
		}
	}
	int resource_count = 0;// resources loaded
	// clearing status
	memset((void*)RESOURCE_THREAD_STATUS, 0, sizeof(RESOURCE_THREAD_STATUS));

	// creating thread pointers
	SDL_Thread * threads[RESOURCE_BATCH_SIZE];
	memset((void*)threads, 0, sizeof(threads));

	std::list<Resource::sThreadData*>::iterator thread_data[RESOURCE_BATCH_SIZE];
	std::list<Resource::sThreadData*>::iterator res_it = t_data.begin();
	SDL_LockMutex(RESOURCE_THREAD_STATUS_MUTEX);
	while(1) {
		int num_threads = 0;
		// wating for every thread that is done
		for(int i = 0; i < RESOURCE_BATCH_SIZE; i++){
			if(threads[i] != NULL && RESOURCE_THREAD_STATUS[i] == 0){
				int status;
				// wait for thread to be done
				SDL_WaitThread(threads[i], &status);
				threads[i] = NULL;
				// loading non-threaded stuff
				if(_resources[(int)(*(thread_data[i]))->id]->load(*(thread_data[i])) || status){
					ERROR_F("Error while loading resource '%s'", (*(thread_data[i]))->path);
					error_count++;
					// freeing resource
					freeResource((*(thread_data[i]))->id);
				}
				// freeing thread data
				delete *(thread_data[i]);
				resource_count++;
				if(callback != NULL)// callback
					callback->updatePercentage((float)resource_count/t_data.size());
			}
			if(threads[i] == NULL && res_it != t_data.end()){ // no thread running in that slot -> get next resource to load 
				createResource((*res_it)->id);// create resource (if not already done)
				Resource * r = _resources[(int)(*res_it)->id];
				(*res_it)->thread_id = i;// setting thread id
				(*res_it)->r = r;// setting this-pointer
				// starting thread
				RESOURCE_THREAD_STATUS[i] = 1;
				INFO_F("-> loading resource: %s", (*res_it)->path);
				threads[i] = SDL_CreateThread(r->getThreadFunction(), "resource decoding thread", (void*)*res_it);
				thread_data[i] = res_it;// save thread data iterator for later
				res_it++;//next resource
			}
		}

		if(resource_count >= t_data.size()){// all resources loaded -> break
			break;
		}

		// wait until some thread signals that it has finished
		SDL_CondWait(RESOURCE_THREAD_FINISHED_SIGNAL, RESOURCE_THREAD_STATUS_MUTEX);
	}
	SDL_UnlockMutex(RESOURCE_THREAD_STATUS_MUTEX);

	if(callback != NULL)
		callback->done(error_count);

	if(error_count > 0)
		ERROR_F("%d resource(s) could not be loaded", error_count);
	else
		INFO_F("Loading Done! (took %f sec)", (SDL_GetTicks()-t)/1000.f);
	return error_count;
}

int ResourceHandler::createResource(RESOURCE_ID id)
{
	const ResourceDescriptor * desc = &RESOURCES_DESCRIPTOR[(int)id];
	if(isLoaded(id)){
		return 1;
	}
	//INFO_F("-> loading %s: %s", Resource::getTypeName(desc->type), desc->path);
	//creating new Resource
	switch(desc->type)
	{
	case RESOURCE_TYPE_IMAGE:
	{
		_resources[(int)id] = new ResourceImage(id);
	}break;
	case RESOURCE_TYPE_SOUND:
	{
		/* */
		WARNING("Sound resource not implemented!");
	}break;
	case RESOURCE_TYPE_FONT:
	{
		_resources[(int)id] = new ResourceFont(id);
	}break;
	default:
	{
		ERROR_F("Invalid resource ID: %d", (int)id);
		return 1;
	}
	}
	_resourcesLoaded ++;
	return 0;
}

void ResourceHandler::freeList(const RESOURCE_ID * list, int size)
{
	INFO("Dropping Resources:");
	for(int i = 0; i < size; i++) {
		freeResource(list[(int)i]);
	}
	INFO("Done!");
}

void ResourceHandler::freeResource(RESOURCE_ID id)
{
	const ResourceDescriptor * desc = &RESOURCES_DESCRIPTOR[(int)id];
	if(_resources[(int)id] != NULL)/* resource was loaded */
	{
		INFO_F("-> dropping %s: %s", Resource::getTypeName(desc->type), desc->path);
		delete(_resources[(int)id]);
		_resources[(int)id] = NULL;
		_resourcesLoaded--;
	}
}

Resource* ResourceHandler::getResource(RESOURCE_ID id)
{
	Resource *r = _resources[(int)id];
	const ResourceDescriptor * desc = &RESOURCES_DESCRIPTOR[(int)id];
	if(r == NULL)//resource has not been loaded before
	{
		ERROR_F("Resource has not been loaded: %s: %s", Resource::getTypeName(desc->type), desc->path);
	}

	return r;
}

ResourceImage* ResourceHandler::getImageResource(RESOURCE_ID id)
{
	if(!checkResourceType(id, RESOURCE_TYPE_IMAGE)){
		return NULL;
	}
	return (ResourceImage*)getResource(id);
}

ResourceFont* ResourceHandler::getFontResource(RESOURCE_ID id)
{
	if(!checkResourceType(id, RESOURCE_TYPE_FONT)){
		return NULL;
	}
	return (ResourceFont*)getResource(id);
}

bool ResourceHandler::checkResourceType(RESOURCE_ID id, int expected_type, bool silent)
{
	const ResourceDescriptor * desc = &RESOURCES_DESCRIPTOR[(int)id];
	if(desc->type != expected_type){
		if(!silent)
			WARNING_F("Expected type %s of resource %s: %s", Resource::getTypeName(expected_type),
			Resource::getTypeName(desc->type), desc->path);
		return false;
	}
	return true;
}

bool ResourceHandler::isLoaded(RESOURCE_ID id)
{
	bool l = _resources[(int)id] != NULL;
	return l;
}
