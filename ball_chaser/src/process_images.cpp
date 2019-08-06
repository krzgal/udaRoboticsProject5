#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget dtt_req;

    dtt_req.request.linear_x = lin_x;
    dtt_req.request.angular_z = ang_z;
	
    if (!client.call(dtt_req)){
        ROS_ERROR("Failed to call service command_robot");
    }	
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int white_pixel_limit = 205; //not so white
    int left_limit = img.width / 3;
    int right_limit = img.width - (img.width / 3);
    int left_center_right_cnt[3] = {0,0,0};

  
    for (int i = 1; i < (img.height * img.width) -1 ; i++) 
    {
 	int posx = i % img.width;
	int posrow = i*3;
	int posrow_last = (i-1)*3;
	int posrow_next = (i+1)*3;
	if(	white_pixel_limit <= img.data[posrow] && 
		white_pixel_limit <= img.data[posrow-1] && 
		white_pixel_limit <= img.data[posrow-2])
	{
	    //mask			
	    if(	(white_pixel_limit <= img.data[posrow_last] && white_pixel_limit <= img.data[posrow_last-1] && white_pixel_limit <= img.data[posrow_last-2]) &&
		(white_pixel_limit <= img.data[posrow_next] && white_pixel_limit <= img.data[posrow_next-1] && white_pixel_limit <= img.data[posrow_next-2]))

	    {
	    	if(posx < left_limit) 
		{
		   left_center_right_cnt[0] += 1;                
	    	}
	    	else if(posx > right_limit) 
	    	{
		   left_center_right_cnt[2] += 1;                
	   	}
	    	else 
	    	{
		   left_center_right_cnt[1] += 1;               
	    	}
	    }
	}
    }

    ROS_INFO("Counts analysis L: %d, C: %d, R: %d", 	left_center_right_cnt[0],
							left_center_right_cnt[1],
							left_center_right_cnt[2]);
    
		
    if(	(left_center_right_cnt[0] == 0) &&
	(left_center_right_cnt[1] == 0) &&
	(left_center_right_cnt[2] == 0))
    {
        drive_robot(0, 0); //stop
    }
    else if( (left_center_right_cnt[0] > left_center_right_cnt[2]) &&
	     (left_center_right_cnt[0] > left_center_right_cnt[1]))
    {
	drive_robot(0.0, 0.25); //left
    }
    else if( (left_center_right_cnt[2] > left_center_right_cnt[0]) &&
	     (left_center_right_cnt[2] > left_center_right_cnt[1]))
    {
	drive_robot(0.0, -0.25); //right
    }
    else 
    {
        drive_robot(0.25, 0); //center
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
