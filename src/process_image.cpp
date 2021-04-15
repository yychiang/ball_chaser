#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <iostream>
#include <algorithm>
#include <vector>

#include <math.h>

using namespace std;
int right_state = 0;
int left_state = 0;
// Define a global client that can request services
ros::ServiceClient client;

double gaussian(double mu, double sigma2, double x)  // Gaussian Function
{
    //Use mu, sigma2 (sigma squared), and x to code the 1-dimensional Gaussian
    double prob = 1.0 / sqrt(2.0 * M_PI * sigma2) * exp(-0.5 * pow((x - mu), 2.0) / sigma2);
    return prob;
}

double gaussianN(double mu, double sigma2, double x)  // Normalized Gaussian Function
{
    //Use mu, sigma2 (sigma squared), and x to code the 1-dimensional Gaussian
    double prob = gaussian(mu,sigma2,x)/gaussian(mu,sigma2,mu);
    return prob;
}



// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float lin_y, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Driving the bot in the specified velocity and direction.");

    // Request specified velocity and direction
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.linear_y = lin_y;
    srv.request.angular_z = ang_z;

    // Call the command_robot service and pass the specified velocity and direction.
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    ROS_INFO_STREAM("Entering the process_image_callback function...");
    int white_pixel = 200;
    int black_pixel = 75;
    
    int height = img.height; // image height, that is, number of rows
    int width = img.width; // image width, that is, number of columns
    int step  = img.step;  // Full row length in bytes
    // The definition of above parameters could be found at 
    // http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html

    int mu_red = 48;
    int mu_green = 168;
    int mu_blue =19;
    double sig2_red = 500;
    double sig2_green = 500;
    double sig2_blue = 470;

    double s_red =0; //similarity of red model
    double s_green =0;
    double s_blue =0;

    double similarity =0; //total similarity of R, G, B. i.e., min(s_red,s_green,s_blue)


    

    vector<int> white_position;  // a vector to store the positions of white pixels
    int positionId; // The position ID of each white pixel we find. 
    // This will be stored in  white_position vector.

    // TODO:
    // (1) Loop through each pixel in the image and check if there's a bright white one
    for (int i = 0; i < height * step; i += 3) {
        positionId = i % ( width * 3 ) / 3;

        s_red = gaussianN(mu_red, sig2_red, img.data[i + 2]);
        s_green = gaussianN(mu_green, sig2_green, img.data[i + 1]);
        s_blue = gaussianN(mu_blue, sig2_blue, img.data[i]);

        similarity = min(min(s_red, s_green), s_blue);
       
        //if ( img.data[i] < 30  && img.data[i + 1] > 110 && img.data[i + 2] < 130 ) {  // img.data[i]: Blue; img.data[i + 1]: Green; img.data[i + 2]: Red
        if (similarity > 0.5){
            // Insert the positionId into white_position vector:
            white_position.push_back(positionId);
        }         
    }
    // (2) Identify if the average of these pixels falls in the left, mid, or right side of the image
    int avg;
    int sum = 0;
    int size = white_position.size();
    
    for(int k=0; k < size; k++){
        sum += white_position[k];            
    }

    if (size == 0){
        // Will request a stop when there's no white ball seen by the camera
        if (right_state == 1 && left_state == 0)
        {
          drive_robot(0.0, 0.0, -1.0);  // This request a right
         }
        else if (right_state == 0 && left_state == 1)
        {
          drive_robot(0.0, 0.0, 1.0);  // This request a left   
        }
        else if (right_state == 1 && left_state == 1)
        {
          drive_robot(0.0, 0.0, 2);  // This request a left  
        }
    }
    else {
        avg = sum / size;

        // Depending on the white ball position, call the drive_bot function and pass velocities to it
        if (avg <= width / 3){
	    drive_robot(0.0, 1.0, 0.0);  // This request should drive my_robot left
            left_state = 1;
            right_state = 0;    
        }
        else if (avg >= 2 * width / 3){
	    drive_robot(0.0, -1.0, 0.0); // This request drives my_robot right
            left_state = 0;
            right_state = 1; 
        }
        else{
	    drive_robot(1.0, 0.0, 0.0);  // This request drives my_robot robot forward
            left_state = 1;
            right_state = 1; 
        }
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
    ROS_INFO_STREAM("Will Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function");
    // ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    ros::Subscriber sub1 = n.subscribe("/raspicam_node/image", 10, process_image_callback);
    //ros::Subscriber sub1 = n.subscribe("/raspicam_node/image/compressed", 10, process_image_callback);
    ROS_INFO_STREAM("After Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function");
    // Handle ROS communication events
    ros::spin();

    return 0;
}
