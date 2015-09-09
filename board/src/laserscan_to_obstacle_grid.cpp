#include "ros/ros.h"
#include "math.h"
#include "sensor_msgs/LaserScan.h"
#include "board/ObstacleGrid.h"

#define ROWS 64
#define COLUMNS 64
#define SQUARE_SIZE 0.25f

//[column][row]

ros::Publisher pub;

void rplidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	board::ObstacleGrid grid;
	grid.cell_size = SQUARE_SIZE;
	grid.column_size = COLUMNS;
	grid.row_size = ROWS;

	//ROS_INFO(msg->range_min.c_str());
        //ROS_INFO(msg->range_max.c_str());

	//Necessary?
	/*
	for (int i = 0; i < columns; i++) {
		for (int j = 0; j < rows; j++) {
			grid->obstacle_grid[i][j] = false;
		}
	}
	*/

	int points = (int) ((msg->angle_max - msg->angle_min) / msg->angle_increment);
	for (int i = 0; i < points; i++) {
		float range = msg->ranges[i];
		if (range > msg->range_min && range < msg->range_max && range < ROWS * SQUARE_SIZE / 2) {
			grid.obstacle_grid[(int) (i * ROWS * (COLUMNS/2 + range * sin(i * msg->angle_increment - msg->angle_min))) + ((int) (COLUMNS/2 + range * cos(i * msg->angle_increment - msg->angle_min)))] = true;
		} 
	}

	pub.publish(grid);
}

int main(int argc, char **argv) {
        ros::init(argc, argv, "laserscan_to_obstacle_grid");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("rplidar", 1000, rplidarCallback);
        pub = n.advertise<board::ObstacleGrid>("obstacles", 1000);
	ros::spin();

        return 0;
}

