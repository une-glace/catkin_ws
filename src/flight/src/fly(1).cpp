#include <ros/ros.h>
#include <cmath>
#include <string.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
enum mode
{
	mission = 0,
	offboard = 1,
	land = 2,
	takeoff = 3,
	none = 4
};
mode mission_mode = mission;

mavros_msgs::State current_state;
void state_sb(const mavros_msgs::State::ConstPtr &msg)
{
	current_state = *msg;
}

geometry_msgs::PoseStamped local_pos;
void local_pos_sb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	local_pos = *msg;
}

inline void quat_to_matrix(geometry_msgs::PoseStamped &pos1, geometry_msgs::PoseStamped &pos2, const float &q, const float &p)
{

	float x = pos1.pose.orientation.x;
	float y = pos1.pose.orientation.y;
	float z = pos1.pose.orientation.z;
	float w = pos1.pose.orientation.w;
	float matrix[3][3];
	matrix[0][0] = 1 - 2 * (y * y + z * z);
	matrix[0][1] = 2 * (x * y - z * w);
	matrix[0][2] = 2 * (x * z + y * w);
	matrix[1][0] = 2 * (x * y + z * w);
	matrix[1][1] = 1 - 2 * (x * x + z * z);
	matrix[1][2] = 2 * (y * z - x * w);
	matrix[2][0] = 2 * (x * z - y * w);
	matrix[2][1] = 2 * (y * z + x * w);
	matrix[2][2] = 1 - 2 * (x * x + y * y);
	Eigen::Matrix3f eigenMatrix;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			eigenMatrix(i, j) = matrix[i][j];
		}
	}
	Eigen::Matrix3f inverseMatrix = eigenMatrix.inverse();
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			matrix[i][j] = inverseMatrix(i, j);
		}
	}
	pos2.pose.position.x = matrix[0][0] * pos1.pose.position.x + matrix[0][1] * pos1.pose.position.y + matrix[0][2] * pos1.pose.position.z;
	pos2.pose.position.y = matrix[1][0] * pos1.pose.position.x + matrix[1][1] * pos1.pose.position.y + matrix[1][2] * pos1.pose.position.z;
	pos2.pose.position.z = pos1.pose.position.z;
	pos2.pose.position.x += q;
	pos2.pose.position.y += p;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rise");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/plane_0/mavros/state", 10, state_sb);
	ros::Subscriber plane_local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/plane_0/mavros/local_position/pose", 1, local_pos_sb);

	ros::Publisher target_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/plane_0/mavros/setpoint_position/local", 10);

	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/plane_0/mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/plane_0/mavros/set_mode");

	ros::Rate rate(20.0);
	ros::spinOnce();
	rate.sleep();

	float target[3] = {400, 0, 30};
	geometry_msgs::PoseStamped pose, trans_pose, fixed_pose;
	pose.pose.position.x = target[0];
	pose.pose.position.y = target[1];
	pose.pose.position.z = target[2];
	/*pose.pose.orientation.w = local_pose.pose.orientation.w;
    pose.pose.orientation.x = local_pose.pose.orientation.x;
    pose.pose.orientation.y = local_pose.pose.orientation.y;
    pose.pose.orientation.z = local_pose.pose.orientation.z;
    const double x=local_pose.pose.position.x;
    const double y=local_pose.pose.position.y;*/

	while (ros::ok() && !current_state.connected)
	{
		ROS_INFO_STREAM_THROTTLE(10, "not connected");
		ros::spinOnce();
		rate.sleep();
	}
	for (int i = 100; ros::ok() && i > 0; --i)
	{ // this step is to enable offboard because of the bug, has no meaning in our code{
		pose.header.stamp = ros::Time::now();
		target_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "AUTO.MISSION";
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	ros::Time last_request = ros::Time::now();

	int count = 0;
	int cnt = 0;
	int ok = 0;
	float error_x = 0;
	float error_y = 0;
	float distance = 0;

	while (ros::ok())
	{
		switch (mission_mode)
		{
		case 0:
			if (!current_state.armed && ok == 0)
			{
				pose.header.stamp = ros::Time::now();
				target_pos_pub.publish(pose);
				if (arming_client.call(arm_cmd) && arm_cmd.response.success)
				{
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
			else if (current_state.armed && ok == 0)
			{
				if (current_state.mode != "AUTO.MISSION")
				{
					pose.header.stamp = ros::Time::now();
					target_pos_pub.publish(pose);
					if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
					{
						ROS_INFO("Mission enabled");
					}
				}
				if (cnt == 0 && local_pos.pose.position.z > 30)
				{
					ROS_INFO_STREAM_THROTTLE(1, "\033[1;32m \033[0m" << pose);
					offb_set_mode.request.custom_mode = "OFFBOARD";
					mission_mode = offboard;
					cnt++;
				}
			}
			else if (ok == 1 && local_pos.pose.position.z > 0)
			{
				if (current_state.mode != "AUTO.MISSION")
				{
					pose.header.stamp = ros::Time::now();
					target_pos_pub.publish(pose);
					if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
					{
						ROS_INFO("Mission enabled");
					}
				}
			}
			else if (ok == 1 && local_pos.pose.position.z <= 0)
			{
				ROS_INFO("Vehicle disarmed");
				ok == 2;
			}
			break;

		case 1:
			if (current_state.mode != "OFFBOARD")
			{

				if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
				{

					ROS_INFO("Offboard enabled");
				}
				else
				{
					ROS_INFO("Offboard disabled");
				}
			}
			error_x = target[0] - local_pos.pose.position.x;
			error_y = target[1] - local_pos.pose.position.y;
			distance = pow(error_x, 2) + pow(error_y, 2);
			pose.pose.position.x = target[0] + error_x / distance / 10;
			pose.pose.position.y = target[1] + error_y / distance / 10;
			ROS_INFO_STREAM_THROTTLE(1, "\033[1;32m \033[0m" << pose);
			ros::spinOnce();
			rate.sleep();
			pose.header.stamp = ros::Time::now();
			target_pos_pub.publish(pose);

			if (count == 0 && abs(local_pos.pose.position.x - target[0]) < 1 && abs(local_pos.pose.position.y - target[1]) < 1)
			{
				ROS_INFO_STREAM_THROTTLE(1, "\033[1;32m complete1\033[0m");
				pose.pose.position.x = target[0] + 300;
				target[0] += 300;
				pose.pose.position.y = target[1];
				count++;
			}
			if (count == 1 && abs(local_pos.pose.position.x - target[0]) < 10 && abs(local_pos.pose.position.y - target[1]) < 10)
			{
				ROS_INFO_STREAM_THROTTLE(1, "\033[1;32m complete2\033[0m");
				pose.pose.position.x = target[0] - 290;
				target[0] -= 290;
				pose.pose.position.y = target[1] - 10;
				target[1] -= 10;
				count++;
			}
			if (count == 2 && abs(local_pos.pose.position.x - target[0]) < 0.5 && abs(local_pos.pose.position.y - target[1]) < 0.5)
			{
				mission_mode = mission;
				offb_set_mode.request.custom_mode = "AUTO.MISSION";
				last_request = ros::Time::now();
				ok = 1;
				break;
			}
			break;
		default:
			last_request = ros::Time::now();
			break;
		}
		ros::spinOnce();
		rate.sleep();

		if (ok == 2)
		{
			break;
		}
	}
	return 0;
}
