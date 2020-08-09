#include "arena.h"

int main(int argc, char ** argv)
{
	/* create and run arena */
	Arena a;
	// if(a.init(argc, argv)){
	// 	return 1;
	// }
	a.init(0,nullptr);
	#ifndef ARENA_USE_ROS
		a.run();
	#else
		a.ros_run();
	#endif

	a.quit();

	return 0;
}
