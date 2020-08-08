/*
 * RessourceHandler.h
 *
 *  Created on: May 19, 2018
 *      Author: zer0divider
 */

#ifndef RESOURCEHANDLER_H_
#define RESOURCEHANDLER_H_

#include "resource.h"
#include <vector>
#include <list>
#include <generated/resources/resource_list.h>

#define RESOURCES_RELOAD 0x00000001 //set flag to reload files in list that are already loaded
#define _RES ResourceHandler::get()
#define _RES_IMAGE(X) (ResourceHandler::get()->getImageResource(X))
#define _RES_SOUND(X) (ResourceHandler::get()->getSoundResource(X))
#define _RES_FONT(X)  (ResourceHandler::get()->getFontResource(X))


// loader callback class for retrieving info about current loading state
class LoadCallback{
public:
	virtual ~LoadCallback(){}
	// called before resource is loaded
	virtual void startLoadingResource(RESOURCE_ID res){}

	// called after resource was loaded
	virtual void doneLoadingResource(RESOURCE_ID res, Uint32 time, bool success){}

	// called after every resource with current percentage of
	virtual void updatePercentage(float percentage){}

	// called if loading is finished
	virtual void done(int errors){}
};

class ResourceHandler : public zTSingleton<ResourceHandler>
{
public:
	ResourceHandler();
	~ResourceHandler();

	void freeAll();

	//load the given objects from file
	//@return number of errors occured while loading
	int loadList(const RESOURCE_ID * list, int size, LoadCallback * callback = NULL, uint32_t flags = 0);

	//free the given objects
	void freeList(const RESOURCE_ID * list, int size);
	void freeResource(RESOURCE_ID id);

	// returns 1 if resource was already loaded, else 0
	int createResource(RESOURCE_ID id);

	//reload everything that has already been loaded
	void reload();

	//acessing resources
	//an error is generated if resource is not loaded yet and NULL will be returned
	Resource* getResource(RESOURCE_ID id);

	// same as getResource(), but type of resource is checked
	ResourceImage* getImageResource(RESOURCE_ID id);

	// same as getResource(), but type of resource is checked
	ResourceFont* getFontResource(RESOURCE_ID id);

	//@return true if resource with id @id was loaded
	bool isLoaded(RESOURCE_ID id);

	// returns false if expected type does not match the type of the given resource
	// if silent = false a warning will be printed
	bool checkResourceType(RESOURCE_ID id, int expected_type, bool silent = false);
private:
	struct LoadPath{
		LoadPath(){paths = NULL; count = 0;}
		~LoadPath(){for(int i = 0; i < count; i++){delete[]paths[i];} delete[]paths;}
		void set(RESOURCE_ID _id, char ** _paths, int _count){id = _id; paths = _paths; count = _count;}
		RESOURCE_ID id;
		char ** paths;
		int count;
	};
	LoadPath _loadPaths[RESOURCES_NUMBER];
	//resources in plain array for quick access
	Resource * _resources[RESOURCES_NUMBER];
	int _resourcesLoaded;
};


#endif /* RESOURCEHANDLER_H_ */
