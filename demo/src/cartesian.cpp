/*********************************************************************
 * Copyright (c) 2019 Bielefeld University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Robert Haschke
   Desc:   Planning a simple sequence of Cartesian motions
*/

#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>

using namespace moveit::task_constructor;

Task createTask() {
	Task t;
	t.stages()->setName("Cartesian Path");

	const std::string group = "interbotix_arm";
	const std::string eef = "interbotix_hand";

	// create Cartesian interpolation "planner" to be used in various stages
	auto cartesian_interpolation = std::make_shared<solvers::CartesianPath>();
	// create a joint-space interpolation "planner" to be used in various stages
	auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

	// start from a fixed robot state
	t.loadRobotModel();
	auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
	{
		auto& state = scene->getCurrentStateNonConst();
		state.setToDefaultValues(state.getJointModelGroup(group), "Home");

		auto fixed = std::make_unique<stages::FixedState>("initial state");
		fixed->setState(scene);
		t.add(std::move(fixed));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("z -0.2", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.z = -0.2;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("x +0.1", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.x = 0.1;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("x -0.2", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.x = -0.2;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("y -0.1", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.y = -0.1;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}

	{  // rotate about TCP
		auto stage = std::make_unique<stages::MoveRelative>("rz +45°", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::TwistStamped twist;
		twist.header.frame_id = "world";
		twist.twist.angular.z = M_PI / 4.;
		stage->setDirection(twist);
		t.add(std::move(stage));
	}

	// {  // rotate about TCP
	// 	auto stage = std::make_unique<stages::MoveRelative>("rz +45°", cartesian_interpolation);
	// 	stage->setGroup(group);
	// 	geometry_msgs::TwistStamped twist;
	// 	twist.header.frame_id = "world";
	// 	twist.twist.angular.z = M_PI / 4.;
	// 	stage->setDirection(twist);
	// 	t.add(std::move(stage));
	// }
	// {  // perform a Cartesian motion, defined as a relative offset in joint space
	// 	auto stage = std::make_unique<stages::MoveRelative>("joint offset", cartesian_interpolation);
	// 	stage->setGroup(group);
	// 	std::map<std::string, double> offsets = { { "panda_joint1", M_PI / 6. }, { "panda_joint3", -M_PI / 6 } };
	// 	stage->setDirection(offsets);
	// 	t.add(std::move(stage));
	// }

	{  // move gripper into predefined open state
		auto stage = std::make_unique<stages::MoveTo>("close gripper", joint_interpolation);
		stage->setGroup(eef);
		stage->setGoal("Closed");
		t.add(std::move(stage));
	}

	{  // move gripper into predefined open state
		auto stage = std::make_unique<stages::MoveTo>("open gripper", joint_interpolation);
		stage->setGroup(eef);
		stage->setGoal("Open");
		t.add(std::move(stage));
	}

	{  // move gripper into predefined open state
		auto stage = std::make_unique<stages::MoveTo>("close gripper", joint_interpolation);
		stage->setGroup(eef);
		stage->setGoal("Closed");
		t.add(std::move(stage));
	}

	{  // move from reached state back to the original state, using joint interpolation
		// specifying two groups (arm and hand) will try to merge both trajectories
		stages::Connect::GroupPlannerVector planners = { { group, joint_interpolation }, { eef, joint_interpolation } };
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		t.add(std::move(connect));
	}

	{  // final state is original state again
		auto fixed = std::make_unique<stages::FixedState>("final state");
		fixed->setState(scene);
		t.add(std::move(fixed));
	}

	return t;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_tutorial");
	// run an asynchronous spinner to communicate with the move_group node and rviz
	ros::AsyncSpinner spinner(1);
	spinner.start();

	auto task = createTask();
	try {
		if (task.plan()) {
			// task.introspection().publishSolution(*task.solutions().front());
			ROS_INFO("Executing solution trajectory");
			moveit_msgs::MoveItErrorCodes execute_result;

			execute_result = task.execute(*task.solutions().front());
			// // If you want to inspect the goal message, use this instead:
			// actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction>
			// execute("execute_task_solution", true); execute.waitForServer();
			// moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
			// task_->solutions().front()->fillMessage(execute_goal.solution);
			// execute.sendGoalAndWait(execute_goal);
			// execute_result = execute.getResult()->error_code;

			if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
				ROS_ERROR_STREAM("Task execution failed and returned: " << execute_result.val);
			}

		} else {
			ROS_ERROR("Planning Failed");
		}
	} catch (const InitStageException& ex) {
		ROS_ERROR("Planning Init failed");
		std::cerr << "planning failed with exception" << std::endl << ex << task;
	}

	ros::waitForShutdown();  // keep alive for interactive inspection in rviz
	return 0;
}

/*********************************************************************
 * Copyright (c) 2019 Bielefeld University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Robert Haschke
   Desc:   Planning a simple sequence of Cartesian motions
*/

// #include <moveit/task_constructor/task.h>

// #include <moveit/task_constructor/stages/fixed_state.h>
// #include <moveit/task_constructor/solvers/cartesian_path.h>
// #include <moveit/task_constructor/solvers/pipeline_planner.h>
// // #include <moveit/task_constructor/solvers/joint_interpolation.h>
// #include <moveit/task_constructor/stages/move_to.h>
// #include <moveit/task_constructor/stages/move_relative.h>
// #include <moveit/task_constructor/stages/connect.h>

// #include <ros/ros.h>
// #include <moveit/planning_scene/planning_scene.h>

// using namespace moveit::task_constructor;

// Task createTask() {
// 	Task t;
// 	t.reset();
// 	t.stages()->setName("Cartesian Path");
// 	t.loadRobotModel();
// 	const std::string group = "interbotix_arm";
// 	const std::string eef = "interbotix_hand";
// 	const std::string hand_frame = "locobot_fingers_link";

// 	// // create Cartesian interpolation "planner" to be used in various stages
// 	// auto cartesian_interpolation = std::make_shared<solvers::CartesianPath>();
// 	// // create a joint-space interpolation "planner" to be used in various stages
// 	// auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

// 	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
// 	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

// 	// Cartesian planner
// 	auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
// 	cartesian_planner->setMaxVelocityScaling(1.0);
// 	cartesian_planner->setMaxAccelerationScaling(1.0);
// 	cartesian_planner->setStepSize(.01);

// 	// Set task properties
// 	t.setProperty("group", group);
// 	t.setProperty("eef", eef);

// 	auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
// 	{
// 		auto& state = scene->getCurrentStateNonConst();
// 		state.setToDefaultValues(state.getJointModelGroup(group), "Home");

// 		auto fixed = std::make_unique<stages::FixedState>("initial state");
// 		fixed->setState(scene);
// 		t.add(std::move(fixed));
// 	}

// 	{
// 		auto stage = std::make_unique<stages::MoveRelative>("z -0.2", cartesian_planner);
// 		// stage->setGroup(group);
// 		stage->properties().set("link", hand_frame);
// 		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
// 		geometry_msgs::Vector3Stamped direction;
// 		direction.header.frame_id = hand_frame;
// 		direction.vector.z = -0.2;

// 		stage->setDirection(direction);
// 		t.add(std::move(stage));
// 	}

// 	{
// 		auto stage = std::make_unique<stages::MoveRelative>("x +0.1", cartesian_planner);
// 		// stage->setGroup(group);
// 		stage->properties().set("link", hand_frame);
// 		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
// 		geometry_msgs::Vector3Stamped direction;
// 		direction.header.frame_id = hand_frame;
// 		direction.vector.x = 0.1;
// 		stage->setDirection(direction);
// 		t.add(std::move(stage));
// 	}

// 	{
// 		auto stage = std::make_unique<stages::MoveRelative>("x -0.2", cartesian_planner);
// 		// stage->setGroup(group);
// 		stage->properties().set("link", hand_frame);
// 		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
// 		geometry_msgs::Vector3Stamped direction;
// 		direction.header.frame_id = hand_frame;
// 		direction.vector.x = -0.2;
// 		stage->setDirection(direction);
// 		t.add(std::move(stage));
// 	}

// 	{
// 		auto stage = std::make_unique<stages::MoveRelative>("y -0.1", cartesian_planner);
// 		// stage->setGroup(group);
// 		stage->properties().set("link", hand_frame);
// 		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
// 		geometry_msgs::Vector3Stamped direction;
// 		direction.header.frame_id = hand_frame;
// 		direction.vector.y = -0.1;
// 		stage->setDirection(direction);
// 		t.add(std::move(stage));
// 	}

// 	{  // rotate about TCP
// 		auto stage = std::make_unique<stages::MoveRelative>("rz +45°", cartesian_planner);
// 		// stage->setGroup(group);
// 		stage->properties().set("link", hand_frame);
// 		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
// 		geometry_msgs::TwistStamped twist;
// 		twist.header.frame_id = hand_frame;
// 		twist.twist.angular.z = M_PI / 4.;
// 		stage->setDirection(twist);
// 		t.add(std::move(stage));
// 	}

// 	// {  // rotate about TCP
// 	// 	auto stage = std::make_unique<stages::MoveRelative>("rz +45°", cartesian_planner);
// 	// 	stage->setGroup(group);
// 	// 	geometry_msgs::TwistStamped twist;
// 	// 	twist.header.frame_id = "world";
// 	// 	twist.twist.angular.z = M_PI / 4.;
// 	// 	stage->setDirection(twist);
// 	// 	t.add(std::move(stage));
// 	// }
// 	// {  // perform a Cartesian motion, defined as a relative offset in joint space
// 	// 	auto stage = std::make_unique<stages::MoveRelative>("joint offset", cartesian_planner);
// 	// 	stage->setGroup(group);
// 	// 	std::map<std::string, double> offsets = { { "panda_joint1", M_PI / 6. }, { "panda_joint3", -M_PI / 6 } };
// 	// 	stage->setDirection(offsets);
// 	// 	t.add(std::move(stage));
// 	// }

// 	{  // move gripper into predefined open state
// 		auto stage = std::make_unique<stages::MoveTo>("close gripper", sampling_planner);
// 		stage->setGroup(eef);
// 		stage->setGoal("Closed");
// 		t.add(std::move(stage));
// 	}

// 	{  // move gripper into predefined open state
// 		auto stage = std::make_unique<stages::MoveTo>("open gripper", sampling_planner);
// 		stage->setGroup(eef);
// 		stage->setGoal("Open");
// 		t.add(std::move(stage));
// 	}

// 	{  // move gripper into predefined open state
// 		auto stage = std::make_unique<stages::MoveTo>("close gripper", sampling_planner);
// 		stage->setGroup(eef);
// 		stage->setGoal("Closed");
// 		t.add(std::move(stage));
// 	}

// 	{  // move gripper into predefined open state
// 		auto stage = std::make_unique<stages::MoveTo>("home gripper", sampling_planner);
// 		stage->setGroup(eef);
// 		stage->setGoal("Home");
// 		t.add(std::move(stage));
// 	}

// 	// {  // move from reached state back to the original state, using joint interpolation
// 	// 	// specifying two groups (arm and hand) will try to merge both trajectories
// 	// 	stages::Connect::GroupPlannerVector planners = { { group, sampling_planner } };
// 	// 	// stages::Connect::GroupPlannerVector planners = { { group, sampling_planner }, { eef, sampling_planner }
// };
// 	// 	auto connect = std::make_unique<stages::Connect>("connect", planners);
// 	// 	connect->properties().configureInitFrom(Stage::PARENT);
// 	// 	t.add(std::move(connect));
// 	// }

// 	// {  // final state is original state again
// 	// 	auto fixed = std::make_unique<stages::FixedState>("final state");
// 	// 	fixed->setState(scene);
// 	// 	t.add(std::move(fixed));
// 	// }

// 	return t;
// }

// int main(int argc, char** argv) {
// 	ros::init(argc, argv, "mtc_tutorial");
// 	// run an asynchronous spinner to communicate with the move_group node and rviz
// 	ros::AsyncSpinner spinner(1);
// 	spinner.start();

// 	auto task = createTask();
// 	try {
// 		if (task.plan(10)) {
// 			ROS_INFO("planned SUCCESS");
// 			// task.introspection().publishSolution(*task.solutions().front());
// 			ROS_INFO("Executing solution trajectory");
// 			moveit_msgs::MoveItErrorCodes execute_result;

// 			execute_result = task.execute(*task.solutions().front());
// 			// // If you want to inspect the goal message, use this instead:
// 			// actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction>
// 			// execute("execute_task_solution", true); execute.waitForServer();
// 			// moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
// 			// task_->solutions().front()->fillMessage(execute_goal.solution);
// 			// execute.sendGoalAndWait(execute_goal);
// 			// execute_result = execute.getResult()->error_code;

// 			if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
// 				ROS_ERROR_STREAM("Task execution failed and returned: " << execute_result.val);
// 			}
// 		} else {
// 			ROS_ERROR("Planning failed");
// 		}
// 	} catch (const InitStageException& ex) {
// 		ROS_ERROR("Planning Init failed");
// 		std::cerr << "planning failed with exception" << std::endl << ex << task;
// 	}

// 	ros::waitForShutdown();  // keep alive for interactive inspection in rviz
// 	return 0;
// }
