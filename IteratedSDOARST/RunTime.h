#ifndef TIMEANALYSIS_H //Avoid duplicate definition
#define TIMEANALYSIS_H

#include <time.h> //analysis run-time

class RunTime
{
public:
	RunTime() : SG_build_part1(0), SG_build_part2(0), SG_build_part3(0), SG_build_part4(0),
					  SG_check_intersection(0), prim_consider_obstacle(0),
					  SG_routing_gen_multi_sources(0), SG_routing_maze_route(0), SG_routing_back_trace(0),SG_routing_restore_variable(0)
					  ,SG_meet_process_entry(0), SG_not_meet_process_entry(0), SG_check_coverage(0), SG_check_ASet(0),
					  program_start(0), program_end(0), others_start(0), others_end(0), post_process_start(0), post_process_end(0),
					  trunk_end(0), trunk_start(0) {
					    slack_assignment_start=0;
					    slack_assignment_end=0;
					    times_of_prim=0;
					    times_of_maze=0;
					  }

	//
	clock_t program_start,program_end;

	//
	clock_t trunk_start,trunk_end;

	//
	clock_t others_start,others_end;	

	//
	clock_t post_process_start,post_process_end;

	//step[0]
	clock_t read_file_start,read_file_end;

	//step[1]
	clock_t SG_build_start,SG_build_end;
		//detail
		float SG_build_part1;
		float SG_build_part2;
		float SG_build_part3;
		float SG_build_part4;

		float SG_check_intersection;


		float SG_meet_process_entry;
		float SG_not_meet_process_entry;

		float SG_check_coverage;
		float SG_check_ASet;

	//step[2]
	clock_t prim_start,prim_end;
		//detail
		float prim_consider_obstacle;

	//step[3]
	clock_t SG_routing_start,SG_routing_end;
		//detail
		float SG_routing_gen_multi_sources;	
		float SG_routing_maze_route;
		float SG_routing_back_trace;
		float SG_routing_restore_variable;

	//step[4]
	clock_t RG_rectilinear_start,RG_rectilinear_end;
	clock_t slack_assignment_start, slack_assignment_end;
	double times_of_prim;
	double times_of_maze;
};

#endif
