###### tags: `ROS`
# Project 2::Section 18 實際機器人的控制
## 18. 實際機器人的控制
觀念上，前面做的兩個package，只會用到`ball_chaser`，不會需要`my_robot`，因為我們將使用真正的TurtleBot3。


TurtleBot3的系統安裝、設定等，都可以在[Robotis官網](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)找到，不再贅述。各位需要做的工作，先摘要如下：
### 18.1 安裝TB3中RPi的Camera驅動程式
安裝TB3中RPi的Camera驅動程式：[raspicam_node](https://github.com/UbiquityRobotics/raspicam_node)
### 18.2 修改`camerav2_1280x960.launch`檔
TB3中設定欲傳送的image的解析度。`raspicam_node`安裝好之後，找到`raspicam_node/launch/`中的`camerav2_1280x960.launch`檔，改名為`camerav2_64x48.launch`，並將內部內容改為：
```xml=
<launch>
  <arg name="enable_raw" default="false"/>
  <arg name="enable_imv" default="false"/>
  <arg name="camera_id" default="0"/>
  <arg name="camera_frame_id" default="raspicam"/>
  <arg name="camera_name" default="camerav2_1280x960"/>

  <node type="raspicam_node" pkg="raspicam_node" name="raspicam_node" output="screen">
    <param name="private_topics" value="true"/>

    <param name="camera_frame_id" value="$(arg camera_frame_id)"/>
    <param name="enable_raw" value="$(arg enable_raw)"/>
    <param name="enable_imv" value="$(arg enable_imv)"/>
    <param name="camera_id" value="$(arg camera_id)"/>

    <param name="camera_info_url" value="package://raspicam_node/camera_info/camerav2_1280x960.yaml"/>
    <param name="camera_name" value="$(arg camera_name)"/>
    <param name="width" value="64"/>
    <param name="height" value="48"/>

    <param name="framerate" value="30"/>
  </node>
</launch>
```

之後要Bringup Camera的時候，可執行：
```
$ roslaunch raspicam_node camerav2_64x48.launch enable_raw:=true
```
### 18.3
PC端使用到image的地方，topic name要修改。例如，process_image.cpp的`main()`中，

```
ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
```

改為：
```
ros::Subscriber sub1 = n.subscribe("/raspicam_node/image", 10, process_image_callback);
```
### 18.4 估計色彩模型
評估真實情況中的色彩模型：模擬中使用的白色球，RGB色彩為$(255,255,255)$，這是理想值。在現實環境中，我們要讓機器人去追某種顏色的球，必須先知道該色彩的RGB數值。新增一個`estimate_color.cpp`如下:
```c=
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <iostream>
#include <algorithm>
#include <vector>
using namespace std;

// Define a global client that can request services
ros::ServiceClient client;



// This callback function continuously executes and reads the image data
void estimate_color_callback(const sensor_msgs::Image img)
{

    int height = img.height; // image height, that is, number of rows
    int width = img.width; // image width, that is, number of columns
    int step  = img.step;  // Full row length in bytes

    vector<int> red;  // a vector to store the value of red component
    vector<int> green;  // a vector to store the value of green component
    vector<int> blue;  // a vector to store the value of green component

    int positionId; 
    

    for (int i = 0; i < height * step; i += 3) {
        positionId = i % ( width * 3 ) / 3;
        if ( (positionId >= width / 3) && (positionId <= 2 * width / 3) ) {
            blue.push_back(img.data[i]);
            green.push_back(img.data[i+1]);
            red.push_back(img.data[i+2]);

        }         
    }

    int avg_red;
    int avg_green;
    int avg_blue;
    int sum_red = 0;
    int sum_green = 0;
    int sum_blue = 0;
    int size = red.size();

    for(int k=0; k < size; k++){
        sum_red += red[k];  
        sum_green += green[k];
        sum_blue += blue[k];           
    }
    avg_red = sum_red / size;
    avg_green = sum_green / size;
    avg_blue = sum_blue / size;
    ROS_INFO("red= %d, green= %d, blue= %d", avg_red, avg_green, avg_blue);
    
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "estimate_color");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    // ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, estimate_color_callback);
    ros::Subscriber sub1 = n.subscribe("/raspicam_node/image", 10, estimate_color_callback);
    // Handle ROS communication events
    ros::spin();

    return 0;
}
```

若使用高斯模型來估計，程式可改為：
```c=
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <iostream>
#include <algorithm>
#include <vector>

#include <math.h>
using namespace std;

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


// This callback function continuously executes and reads the image data
void estimate_color_callback(const sensor_msgs::Image img)
{

    int height = img.height; // image height, that is, number of rows
    int width = img.width; // image width, that is, number of columns
    int step  = img.step;  // Full row length in bytes

    vector<int> red;  // a vector to store the value of red component
    vector<int> green;  // a vector to store the value of green component
    vector<int> blue;  // a vector to store the value of green component

    int positionId; 
    

    for (int i = 0; i < height * step; i += 3) {
        positionId = i % ( width * 3 ) / 3;
        if ( (positionId >= width / 3) && (positionId <= 2 * width / 3) ) {
            blue.push_back(img.data[i]);
            green.push_back(img.data[i+1]);
            red.push_back(img.data[i+2]);

        }         
    }

    int avg_red;
    int avg_green;
    int avg_blue;
    int sum_red = 0;
    int sum_green = 0;
    int sum_blue = 0;

    int ssum_red = 0; //squared sum
    int ssum_green = 0;
    int ssum_blue = 0;

    double savg_red = 0; //squared average
    double savg_green = 0;
    double savg_blue = 0;

    double sig2_red = 0;   // variance of red component
    double sig2_green = 0;
    double sig2_blue = 0;


    int size = red.size();

    for(int k=0; k < size; k++){
        sum_red += red[k];  
        sum_green += green[k];
        sum_blue += blue[k];         

        ssum_red += red[k] * red[k];
        ssum_green += green[k] * green[k];
        ssum_blue += blue[k] * blue[k];  
    }
    avg_red = sum_red / size;
    avg_green = sum_green / size;
    avg_blue = sum_blue / size;

    savg_red = ssum_red / size;
    savg_green = ssum_green / size;
    savg_blue = ssum_blue / size;

    sig2_red = savg_red - avg_red * avg_red;
    sig2_green = savg_green - avg_green * avg_green;
    sig2_blue = savg_blue - avg_blue * avg_blue;


    ROS_INFO("red= %d %f, green= %d %f, blue= %d %f", avg_red, sig2_red, avg_green, sig2_green, avg_blue, sig2_blue);
    
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "estimate_color");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    // ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, estimate_color_callback);
    ros::Subscriber sub1 = n.subscribe("/raspicam_node/image", 10, estimate_color_callback);
    // Handle ROS communication events
    ros::spin();

    return 0;
}

```

### 18.5 修改`CMakeLists.txt`
最後面加上：
```cmake=
add_executable(estimate_color src/estimate_color.cpp)
add_dependencies(estimate_color ball_chaser_generate_messages_cpp)
target_link_libraries(estimate_color ${catkin_LIBRARIES})
```

```bash
$ cd ~/catkin_ws/
$ catkin_make
```

### 18.6 使用Mecanum wheel
#### Service要改
因為麥克輪可以做橫向運動，因此加入$y$方向的線性運動。
`DriveToTarget.srv`如下：
```
float64 linear_x
float64 linear_y
float64 angular_z
---
string msg_feedback
```

#### drive_bot.cpp的修改
加入Line 21，修改Line 27。
```c=
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//TODO: Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
{
    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;

    // Publish angles to drive the robot
    motor_command.linear.x = req.linear_x;
    motor_command.linear.y = req.linear_y;
    motor_command.angular.z = req.angular_z;
    
    motor_command_publisher.publish(motor_command);

    // Return a response message
    res.msg_feedback = "Velocities set - linear_x: " + std::to_string(req.linear_x) + ", Velocities set - linear_y: " + std::to_string(req.linear_y) + " , angular_z: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);
    
    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");
    ROS_INFO_STREAM("Initialize a ROS node named drive_bot.");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}
```
測試：
```bash
$ rosservice call /ball_chaser/command_robot "linear_x: 0.0
linear_y: 0.5
angular_z: 0.0"  # left

$ rosservice call /ball_chaser/command_robot "linear_x: 0.0
linear_y: -0.5
angular_z: 0.0"  # right

$ rosservice call /ball_chaser/command_robot "linear_x: 0.5
linear_y: 0.5
angular_z: 0.0"  # forward-left

$ rosservice call /ball_chaser/command_robot "linear_x: 0.5
linear_y: -0.5
angular_z: 0.0"  # forward-right

$ rosservice call /ball_chaser/command_robot "linear_x: 0.0
linear_y:  0.0
angular_z: 0.0" # Stop!!
```
#### process_image.cpp的修改
加入Line 41，修改Line 33, 113, 117, 121, 129, 134, 139。
```c=
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
```
