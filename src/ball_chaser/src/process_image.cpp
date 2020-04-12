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
    int white_pixel = 255*3;
    int pix_sum = 0;
    int num_white_row = 0;
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
            for (int k = 0; k < 3; k++){ // look at rgb values
                pix_sum += img.data[3*i + j*step + k];
            }

            /* store last white pixel detected in image 
            left to right, top to bottom */
            if (pix_sum == white_pixel){
                pix_x = i;
                num_white++;
                num_white_row++;
            }
            pix_sum = 0;
        }

        // store size of largest row of white pixels
        if (num_white_row > pix_len){
            pix_len = num_white_row;
        }
        num_white_row = 0;
    }

    // ratio of white pixels to others
    cov = ((float)(100*num_white))/((float)(h*w));

    /* readjust x pos of white ball to centre pos of white ball
    this is only an approximation, does not take ball curvature
    into account */
    pix_x = pix_x - (pix_len/2);

    // magic tuned paramters here
    if (cov > 0.01){ // for robustness
        if (pix_x <= (w/3)){
            z = 1;
            x = 0.1/cov;
	    ROS_INFO_STREAM("Left");
        }
        else if (((w/3) < pix_x) && (pix_x <= (2.0*w/3.0))){
            z = 0.0;
            x = 0.1/cov;
	    ROS_INFO_STREAM("Straight");
        }
        else if (pix_x > (2*w/3.0)){
            z = -1;
            x = 0.1/cov;
	    ROS_INFO_STREAM("Right");
        }
        else{
            z = 0.0;
            x = 0.0;
        }
    }
    else{
        z = 0.0;
        x = 0.0;
    }

    // set limit on max linear speed
    if (x > 0.4){
        x = 0.4;
    }
    
    drive_robot(x, z);
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
