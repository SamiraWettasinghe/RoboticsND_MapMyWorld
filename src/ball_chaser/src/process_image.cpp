#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient client;

void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv)){
        ROS_ERROR("Failed to call service command_robot");
    }
}

void process_image_callback(const sensor_msgs::Image img)
{
    // init local temp variables
    int white_pixel = 255;
    int pix_sum = 0;
    int num_white = 0;
    int pix_x = 0;
    int pix_len = 0;
    int k = 0;
    float cov = 0.0;
    float x = 0.0;
    float z = 0.0;

    // image payload and header
    int step = img.step;
    int h = img.height;
    int w = img.width;

    for (int j = 0; j < h; j++){
        for (int i = 0; i < w; i++){
            for (int k = 0; k < 3; k++){
                pix_sum += img.data[3*i + j*step + k];
            }

            if (pix_sum == white_pixel){
                pix_x = i;
                num_white++;
            }
            pix_sum = 0;
        }
        
        if (num_white > pix_len){
            pix_len = num_white;
        }
        num_white = 0;
    }

    pix_x = pix_x - (pix_len/2);

    if (pix_x <= (w/3)){
        z = 0.5;
        x = 0.5;
	ROS_INFO_STREAM("Left");
    }
    else if (((w/3) < pix_x) && (pix_x <= (2*w/3))){
        z = 0.0;
        x = 0.5;
	ROS_INFO_STREAM("Straight");
    }
    else if (pix_x > (2*w/3)){
        z = -0.5;
        x = 0.5;
	ROS_INFO_STREAM("Right");
    }
    else{
        z = 0.0;
        x = 0.0;
    }
    
    drive_robot(x, z);

    /* 
       later:
       determine the projection of the white circle on the x and y axis.
       this will give a very accurate prediction of the centre location
       of the sphere.
       
       linear velocity will be scaled by how far away ball if from centre,
       far away from centre = slow vel.
       angular velociy will be scaled by white pixel ratio,
       small ratio = slow vel.
       
       if x and y projections are not equal in size, that means ball
       is partially off camera.
       if this is the case, rotate camera slowly with zero linear vel
       to get full image of ball in view.
    */
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    ros::spin();

    return 0;
}
