#include <ros/ros.h>
#include <vector>
#include <armadillo>
#include <cmath>
// PCL includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// tf2 includes
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/message_filter.h>
// cv includes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


static ros::Publisher cloud_pub;
static ros::Publisher image_pub;
static std::string cam_frame_id = "front_realsense_gazebo";
static std::string cloud_frame_id = "velodyne";
static tf2_ros::Buffer tf_buffer;
static sensor_msgs::Image sensor_image;
namespace enc = sensor_msgs::image_encodings;
static cv::Mat cv_image;
static cv_bridge::CvImageConstPtr cv_ptr;
static arma::mat projection = 
{
    {337.2084410968044, 0.0, 320.5},
    {0.0, 337.2084410968044, 240.5},
    {0.0, 0.0, 1.0}
};

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        cv_image = cv_ptr->image;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // for (int r = 0; r < cv_ptr->image.rows; r++)
    // {
    //     for (int c = 0; c < cv_ptr->image.cols; c++)
    //     {
    //         int b = cv_ptr->image.at<cv::Vec3b>(r,c)[0];
    //         ROS_INFO("Row: %i, Col: %i, Blue: %i \n", r,c,b);
    //         // ROS_INFO("Row: %i, Col: %i\n", r,c);
    //     }
    // }

    // image_pub.publish(cv_ptr->toImageMsg());
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Create a container for the output data.
    pcl::PointCloud<pcl::PointXYZRGB> cloud_out;

    // Get transform from cloud frame to camera frame
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = input->header.frame_id;
    transform.header.stamp = input->header.stamp;
    // transform.transform.translation.x = 0.195;
    // transform.transform.translation.y = 0.00;
    // transform.transform.translation.z = -0.128;
    // transform.transform.rotation.x = -0.500;
    // transform.transform.rotation.y =  0.500;
    // transform.transform.rotation.z = -0.500;
    // transform.transform.rotation.w =  0.500;

    transform.transform.translation.x = 0.000;
    transform.transform.translation.y = -0.128;
    transform.transform.translation.z = -0.195;
    transform.transform.rotation.x = 0.500;
    transform.transform.rotation.y = -0.500;
    transform.transform.rotation.z = 0.500;
    transform.transform.rotation.w =  0.500;

    // try
    // {
    //     transform = tf_buffer.lookupTransform(image.header.frame_id,input->header.frame_id, input->header.stamp);
    // }
    // catch(const tf2::TransformException& e)
    // {
    //     ROS_WARN_STREAM("LookupTransform failed. Reason: " << e.what());
    // }

    // Transform point cloud to camera frame
    sensor_msgs::PointCloud2 cloud_cam;
    tf2::doTransform(*input, cloud_cam, transform);
    cloud_cam.header.frame_id = cam_frame_id;

    // Convert sensor msg to pcl
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(cloud_cam,cloud);

    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::fromROSMsg(*input,cloud_in);

    

    // Iterate over each point in cloud
    for (unsigned int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i].z>0)
        {
            arma::mat point = arma::colvec( {cloud[i].x, cloud[i].y, cloud[i].z} );
            // point.print("Point:");
            // projection.print("Projection:");
            // arma::mat point = arma::mat(4,1,arma::fill::ones);
            // point(0,0) = cloud[i].x;
            // point(1,0) = cloud[i].y;
            // point(2,0) = cloud[i].z;        
            // arma::mat point = arma::mat(4,1,arma::fill::ones);
            // point(0,0) = cloud[i].x;
            // point(1,0) = cloud[i].y;
            // point(2,0) = cloud[i].z;
            // arma::mat camera_coords;
            arma::mat camera_coords = (projection*point)/cloud[i].z;
            int x_coord = round(camera_coords(0,0));
            int y_coord = round(camera_coords(1,0));
            if (x_coord>=0 && x_coord<640 && y_coord>=0 && y_coord<480)
            {
                // ROS_INFO("x_coord: %i, y_coord: %i",x_coord,y_coord);
                pcl::PointXYZRGB colored_point;
                colored_point.x = cloud[i].x;
                colored_point.y = cloud[i].y;
                colored_point.z = cloud[i].z;
                // try
                // {
                //     int r = cv_image.at<cv::Vec3b>(100,100)[2];
                //     // int g = cv_image.at<cv::Vec3b>(0,0)[1];
                //     // int b = cv_image.at<cv::Vec3b>(0,0)[0];
                //     // ROS_INFO("r: %i, g: %i, b: %i",r,g,b);
                //     ROS_INFO("x: %i y: %i r: %i",x_coord,y_coord,r);
                // }
                // catch(const std::exception& e)
                // {
                //     ROS_ERROR("exception: %s", e.what());
                // }
                int r = cv_image.at<cv::Vec3b>(y_coord,x_coord)[2];
                int g = cv_image.at<cv::Vec3b>(y_coord,x_coord)[1];
                int b = cv_image.at<cv::Vec3b>(y_coord,x_coord)[0];
                
                colored_point.r = r;
                colored_point.g = g;
                colored_point.b = b;
                colored_point.a = 255;
                cloud_out.push_back(colored_point);
            }

            // pcl::PointXYZRGB colored_point;
            // colored_point.x = cloud[i].x;
            // colored_point.y = cloud[i].y;
            // colored_point.z = cloud[i].z;
            // colored_point.r = 0;
            // colored_point.g = 100;
            // colored_point.b = 100;
            // colored_point.a = 255;
            // cloud_out.push_back(colored_point);
        }
    }

    // Convert pcl to sensor msg
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_out,output);
    output.header.stamp = input->header.stamp;
    output.header.frame_id = cam_frame_id;


    // Publish the data.
    cloud_pub.publish (output);
}

void print_image()
{
    for (int r = 0; r < cv_image.rows; r++)
    {
        for (int c = 0; c < cv_image.cols; c++)
        {
            int b = cv_image.at<cv::Vec3b>(r,c)[0];
            ROS_INFO("Row: %i, Col: %i, Blue: %i \n", r,c,b);
            // ROS_INFO("Row: %i, Col: %i\n", r,c);
        }
    }
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "color_cloud");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input image
    ros::Subscriber image_sub = nh.subscribe("/realsense/color/image_raw", 1, image_cb);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber cloud_sub = nh.subscribe("/mid/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/output/points", 1);

    // Create a Ros publisher for the output image
    image_pub = nh.advertise<sensor_msgs::Image>("/output/image", 1);


    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        // print_image();
        ros::spinOnce();
        loop_rate.sleep();
    }

}