/************************************************************
Project   : objectLocator.cpp
Created By: Curtis Muntz
Date      : 19 July 2014
Compile   : Use catkin_make to compile.
Usage     : rosrun production objectLocator
************************************************************
Concept   : This is a heavily modified image_converter file that does the following:
              1) converts raw image into opencv image
              2) isolates orange in the image
              3) reduces largest orange object to a single point
              4) publishes the center of the object to "objectLocation"

Requires  : OpenCV, ROS installed
          : rosnode "image_sender" needs to be running (or listening to another "camera/image_raw")
TODO      : refine object detection...brown is too orange
*************************************************************/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/UInt16.h"


#define DEBUG false
#define OUTPUT_LOCATION_TOPIC "objectLocation"
#define OUTPUT_AREA_TOPIC "objectArea"
//#define CAMERA_IMAGE_TOPIC "/lifecam/image_raw"
//#define CAMERA_IMAGE_TOPIC "/image_raw"
#define CAMERA_IMAGE_TOPIC "/camera/image_raw"

static const std::string OPENCV_WINDOW = "objectLocator window";

/****func declarations****/
cv::Mat isolateOrange(const cv::Mat& src);
cv::Rect objectBounding(const cv::Mat& src);


/*****Image Converter Class*****/
class ImageConverter
{
  ros::NodeHandle nh_;
  
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  ros::Publisher location_pub;
  ros::Publisher area_pub;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(CAMERA_IMAGE_TOPIC, 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    

    cv::namedWindow(OPENCV_WINDOW);
  }

  /*****deconstructer *****/
  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  /*****created function for asdvertise*****/
  void advertise_location(const char *topic_name)
  {
    location_pub = nh_.advertise<std_msgs::UInt16>(topic_name, 1000);
  }

  /*****created this function to send topic*****/
  void publish_location(std_msgs::UInt16 msg)
  {
    //publish the message
    location_pub.publish(msg);
  }





    /*****created function for asdvertise*****/
  void advertise_area(const char *topic_name)
  {
    area_pub = nh_.advertise<std_msgs::UInt16>(topic_name, 1000);
  }

  /*****created this function to send topic*****/
  void publish_area(std_msgs::UInt16 msg)
  {
    //publish the message
    area_pub.publish(msg);
  }







  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


    /***** Isolate orange *****/
    cv::Mat frame = cv_ptr->image;
    //blur dat img NOTE ORIGINAL TESTING DID NOT HAVE THIS
    cv::Mat orangeOnly = isolateOrange(frame);


    /***** get the bounding box of the orange image (using contours) *****/
    cv::Rect bounded_object = objectBounding(orangeOnly);
    cv::rectangle(frame, bounded_object,  cv::Scalar(0,255,0),2, 8,0); //draw the box

   /*****find center of bounding box*****/
   int x_coord = bounded_object.x + bounded_object.width/2;
   int y_coord = bounded_object.y + bounded_object.height/2;

   int area = bounded_object.width * bounded_object.height;

   /***** publish center *****/
   std_msgs::UInt16 out_msg;
   std_msgs::UInt16 area_msg;
   out_msg.data = x_coord;
   area_msg.data = area;

   advertise_location(OUTPUT_LOCATION_TOPIC);
   publish_location(out_msg);
   advertise_area(OUTPUT_AREA_TOPIC);
   publish_area(area_msg);

   if (DEBUG == true)
   {
      std::cout << "x:   " << x_coord << " y:   " << y_coord << std::endl;
      ROS_INFO("%i", out_msg.data);
   }
   

    // Draw the center of the object as a circle on the video
    cv::circle(frame, cv::Point(x_coord, y_coord), 10, cv::Scalar(255,255,0));



    cv::imshow(OPENCV_WINDOW, frame);
    cv::waitKey(3);
    
    // Output modified video stream (not really necessary but i copied this base code from image_converter)
    image_pub_.publish(cv_ptr->toImageMsg());
  }





};

cv::Mat isolateOrange(const cv::Mat& src)
{
  cv::Mat frame = src.clone();


  cv::Mat frameycbcr;
  cv::cvtColor(frame,frameycbcr,CV_BGR2YCrCb);
  /*****isolate cr channel*****/
  cv::vector<cv::Mat> channels;
  split(frameycbcr, channels);
  cv::Mat Cr = channels[1];
  cv::Mat Cb = channels[2];




  /*****threshold to black/white*****/
  double thresh = 0.57;
  cv::Mat Bw = Cr > (thresh*255);
  if(DEBUG){
    cv::imshow("B/W", Bw);
    cv::imshow("source", frame);
    cv::imshow("ycbcr", frameycbcr);
    cv::imshow("Cr", Cr);
    cv::imshow("Cb", Cb);
  }

  return Bw;
}



cv::Rect objectBounding(const cv::Mat& src)
{

    //Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
    std::vector<std::vector<cv::Point> > contours;
    cv::Mat contourOutput = src.clone();
    cv::findContours( contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

    //Draw the contours
    cv::Mat contourImage = cv::Mat::zeros(src.size(), CV_8UC3); //create black image for drawing contours

    for (size_t idx = 0; idx < contours.size(); idx++) {
        //cv::drawContours(contourImage, contours, idx, cv::Scalar(255,255,0));
        cv::drawContours(contourImage, contours, idx, cv::Scalar(255,255,0), CV_FILLED);
    }
    
    //find largest contour
    int largest_area=0;
    int largest_contour_index=0;
    cv::Rect bounding_rect;

    for( int i = 0; i< contours.size(); i++ )
    {
        //  Find the area of contour
        double a=contourArea( contours[i],false); 
        if(a>largest_area)
        {
            largest_area=a;
            // Store the index of largest contour
            largest_contour_index=i;               
            // Find the bounding rectangle for biggest contour
            bounding_rect=boundingRect(contours[i]);
        }
    }

    if(DEBUG){
      cv::imshow("Input Image", src);
      cv::imshow("Contours", contourImage);
    }
    return bounding_rect;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "objectLocator");
  ros::NodeHandle nh_;
  
  ImageConverter ic;

  ros::spin();
  return 0;
}