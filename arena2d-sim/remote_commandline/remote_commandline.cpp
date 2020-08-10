#include <ros/ros.h>
#include <arena2d/arenaCommand.h>
#include "arena/CommandStatus.h"

int main(int argc, char ** args)
{
	ros::init(argc, args, "remote_commandline");
	ros::NodeHandle n;

	ros::ServiceClient c = n.serviceClient<arena2d::arenaCommand>("arena_command");
	if(!c){
		ROS_ERROR("Unable to subscribe to service 'arena_command'!");
		return 1;
	}
	arena2d::arenaCommand srv;

	printf("Type a command to run remotely in the arena.\nType 'exit' to quit this program.\n");
	while(1){
		printf("> ");
		getline(std::cin, srv.request.command);
		if(srv.request.command.empty())// skip empty commands
			continue;
		if(srv.request.command == "exit")
			break;
		ros::Time t = ros::Time::now();
		if(c.call(srv)){
			ros::Duration d = (ros::Time::now()-t);
			CommandStatus status = (CommandStatus)srv.response.result;
			switch(status){
			case CommandStatus::SUCCESS:
				fprintf(stderr, "Success!\n");
			break;
			case CommandStatus::EXEC_FAIL:
				fprintf(stderr, "ERROR: Failed to execute command!\n");
			break;
			case CommandStatus::INVALID_ARG:
				fprintf(stderr, "ERROR: Invalid arguments!\n");
			break;
			default:
				fprintf(stderr, "ERROR: Unknown command!\n");
			}
			printf("(call took %.3f seconds)\n", d.toSec());

		}
		else{
			ROS_ERROR("Failed to call service 'arena_command'!");
		}
		puts(" ");

	}

	return 0;
}
