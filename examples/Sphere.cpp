/*
 * Sphere.cpp
 * RVO2-3D Library
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

/* Example file showing a demo with 812 agents initially positioned evenly distributed on a sphere attempting to move to the antipodal position on the sphere. */

#ifndef RVO_OUTPUT_TIME_AND_POSITIONS
#define RVO_OUTPUT_TIME_AND_POSITIONS 1
#endif

#include <cmath>
#include <cstddef>
#include <vector>
#include <sstream>

#if RVO_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif

#ifdef _OPENMP
#include <omp.h>
#endif


#include "RVO.h"
#include "Agent.h"
#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif

/* Store the goals of the agents. */
std::vector<RVO::Vector3> goals;
std::vector<RVO::Agent*> agents_;
float time_step;
float globalTime_;

void addAgent(const RVO::Vector3& position, RVO::Vector3 goal)
{
	RVO::Agent* agent = new RVO::Agent();

	agent->position_ = position;
	//float neighborDist, size_t maxNeighbors, float timeHorizon, 
	//float radius, float maxSpeed, const Vector3& velocity = Vector3()
	agent->setAgentDefaults(5.0f, 10, 10.0f, 1.5f, 2.0f,RVO::Vector3(1,1,1));//velocity

	agent->id_ = agents_.size();
	/* Specify the global time step of the simulation. */
	agent->setTimeStep(1.0f);

	agent->agent_goal = goal;
	agents_.push_back(agent);

}

void setupScenario()
{
	time_step = 1.0f;
	globalTime_ = 0;

	int num = 12;
	float x_list[12] = { 36.0 ,30.0 ,16.0 ,-2.1,-20.0,-35.0,-40.0, -35.0 ,-20.0,-2.1,16.0,30.6 };//, -3.0, -3.0, -30.7, 30.1};//
	float y_list[12] = { 5.0 ,22.4 ,34.0,37.7 ,34.0,22.4,5.0,-12.4,-24.0 ,-27.7 ,-24.0 ,-12.4 };// , 20.8, -10.5, 6.0, 6.0 };//};
	/* circle 12
	3.659 0.498 0.000
	3.063 2.244 0.000
	1.604 3.390 0.000
	-0.216 3.773 0.000
	-2.035 3.390 0.000
	-3.495 2.244 0.000
	-4.091 0.498 0.000
	-3.495 -1.247 0.000
	-2.035 -2.393 0.000
	-0.216 -2.777 0.000
	1.604 -2.393 0.000
	3.063 -1.247 0.000
	*/

	// square 4
	/*
	-5.591 4.098 0.000
	5.534 4.098 0.000
	5.534 -3.702 0.000
	-5.591 -3.702 0.000
	*/

	/* circle 4
	4.809 -0.164 0.000
	-0.316 3.998 0.000
	-5.441 -0.164 0.000
	-0.316 -4.327 0.000

	*/
	/* circle 8
	0.309 1.623 0.000
	-0.515 3.176 0.000
	-2.216 3.723 0.000
	-3.916 3.176 0.000
	-4.741 1.623 0.000
	-3.916 0.071 0.000
	-2.216 -0.477 0.000
	-0.515 0.071 0.000
	*/
	/* square 8
	-5.216 3.623 0.000
	5.434 3.623 0.000
	5.434 -2.402 0.000
	-5.216 -2.402 0.000
	0.109 3.623 0.000
	0.109 -2.402 0.000
	-5.216 0.611 0.000
	5.434 0.611 0.000
	*/
	/* square 12
	-6.016 3.448 0.000
	5.234 3.448 0.000
	5.234 -2.002 0.000
	-6.016 -2.002 0.000
	-3.203 3.448 0.000
	-3.203 -2.002 0.000
	-0.391 3.448 0.000
	-0.391 -2.002 0.000
	2.422 3.448 0.000
	2.422 -2.002 0.000
	-6.016 0.723 0.000
	5.234 0.723 0.000
	*/
	float x_goal_list[12] = { -60.0 ,52.3 ,52.3,-60.0,-32.0,-32.0,-4.0,-4.0, 24.2, 24.2, -60.0, 52.3 };// , -30.4, -20.5, -5, 10.4};// };
	float y_goal_list[12] = { 34.4,34.4,-20.0,-20.0,34.4,-20.0,34.4,-20.0,34.4,-20.0,7.2,7.2 };//,3.0,-10.5,-20.2,-10.5 };//};

	for (int i = 0; i < num; i++)
	{
		RVO::Vector3 pos = RVO::Vector3(x_list[i], y_list[i], 0);
		RVO::Vector3 goal = RVO::Vector3(x_goal_list[i], y_goal_list[i], 0);
		addAgent(pos, goal);

		goals.push_back(goal);
	}
	/* Add agents, specifying their start position, and store their goals on the opposite side of the environment. 
	for (float a = 0; a < M_PI; a += 0.1f) {
		const float z = 100.0f * std::cos(a);
		const float r = 100.0f * std::sin(a);

		for (size_t i = 0; i < r / 2.5f; ++i) {
			const float x = r * std::cos(i * 2.0f * M_PI / (r / 2.5f));
			const float y = r * std::sin(i * 2.0f * M_PI / (r / 2.5f));

			RVO::Vector3 pos = RVO::Vector3(x, y, z);
			addAgent(pos, -pos);
			
			goals.push_back(-pos);
		}
	}*/
}

#if RVO_OUTPUT_TIME_AND_POSITIONS
void updateVisualization()
{
	/* Output the current global time. */
	//for (int i = 0; i < agents_.size(); i++)
	//{
	//	if (agents_[i].reached == false) {
	//	std::cout << i << ": "<<agents_[i].getGlobalTime() << std::endl;
	//	std::cout << "position: " << agents_[i].position_ << std::endl;
	//	}
	//	
	//}
	//std::cout << "--------------------------------------------------\n";
	//std::cout << std::endl;

	/* Output the current global time. */
	std::cout << globalTime_;
	globalTime_ += time_step;

	/* Output the position for all the agents. */
	for (size_t i = 0; i < agents_.size(); ++i) {
		std::cout << " " << agents_[i]->position_;
	}

	std::cout << std::endl;
}
#endif

void setPreferredVelocities()
{
	/* Set the preferred velocity to be a vector of unit magnitude (speed) in the direction of the goal. */
	for (size_t i = 0; i < agents_.size(); ++i) {

		RVO::Vector3 goalVector = goals[i] - agents_[i]->position_;

		if (RVO::absSq(goalVector) > 1.0f) {
			goalVector = RVO::normalize(goalVector);
		}
		agents_[i]->setAgentPrefVelocity(goalVector);
	}
}


int main()
{

	/* Set up the scenario. */
	//std::cout << "hi\n";
	setupScenario();

	/* Perform (and manipulate) the simulation. */
	std::cout << agents_.size()<<std::endl;
	int count = 0;
	while (count < agents_.size())
	{

#if RVO_OUTPUT_TIME_AND_POSITIONS
		updateVisualization();
#endif
		setPreferredVelocities();



#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int j = 0; j < agents_.size(); j++)
		{
			//std::cout << agents_[j]->reached <<"hi\n";
			if (agents_[j]->reached == false)
			{
				//std::cout << agents_[j]->id_ << " run \n";
				agents_[j]->run(agents_);
				
				//std::cout << " run \n";

			}
		}
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int j = 0; j < agents_.size(); j++)
		{
			if (agents_[j]->reached == false)
			{
				agents_[j]->update();
				if (agents_[j]->reached == true)
				{
					//std::cout << "------ agent[" << agents_[j]->id_ << "] reached -----\n";
					count += 1;
				}
			}
		}

	}
	//std::cout << "count\n";
	for (int j = 0; j < agents_.size(); j++)
	{
		delete agents_[j];
	}
	
	return 0;
}
