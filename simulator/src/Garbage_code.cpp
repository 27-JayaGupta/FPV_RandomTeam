    #include <ros/ros.h>
    #include <image_transport/image_transport.h>
    #include <cv_bridge/cv_bridge.h>
    #include <sensor_msgs/image_encodings.h>
    #include <opencv2/imgproc/imgproc.hpp>
    #include <opencv2/highgui/highgui.hpp>

    // #include <opencv2/opencv.hpp>
    #include <iostream>
    #include <string>
    #include <numeric>
    int lowThreshold = 80;
    const int max_lowThreshold = 200;
    const int ratio = 3;
    const int kernel_size = 3;
    const double scale=1;
    const char* window_name = "Edge Map";
    const int corner_threshold=100;
    const double epsilon=1;
    
    static const std::string OPENCV_WINDOW = "Image window";

   class ImageConverter
   {
     ros::NodeHandle nh_;
     image_transport::ImageTransport it_;
     image_transport::Subscriber image_sub_;
     image_transport::Publisher image_pub_;
   
   public:
     ImageConverter()
       : it_(nh_)
     {
       // Subscrive to input video feed and publish output video feed
       image_sub_ = it_.subscribe("/iris/camera_red_iris/image_raw", 1,
         &ImageConverter::imageCb, this);
       image_pub_ = it_.advertise("/image_converter/output_video", 1);
   
       cv::namedWindow(OPENCV_WINDOW);
     }
   
     ~ImageConverter()
     {
       cv::destroyWindow(OPENCV_WINDOW);
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
   
        // Update GUI Window
       cv::Mat img=cv_ptr->image;

       cv::imshow(OPENCV_WINDOW, img);

       //Image Manipulation logic
        cv::resize(img, img, cv::Size(img.cols*scale, img.rows*scale));
        cv::imshow("Original", img);
        cv::Mat hsv;
        cv::cvtColor(img,hsv,CV_BGR2HSV);

        std::vector<cv::Mat> channels;
        cv::split(hsv, channels);

        cv::Mat H = channels[0];
        cv::Mat S = channels[1];
        cv::Mat V = channels[2];
        cv::GaussianBlur(S, S, cv::Size(15,15), 0, 0);
        cv::Mat grey,dst,detected_edges;
        cv::cvtColor(img, grey, CV_BGR2GRAY);
        dst.create( img.size(), img.type() );
        cv::imshow("S channel", S);
        cv::Mat canny_output;
        cv::Canny( S, canny_output, lowThreshold, lowThreshold*ratio, kernel_size );
        cv::GaussianBlur(canny_output, canny_output, cv::Size(3,3), 0, 0);

        cv::imshow("Canny Output", canny_output);
        cv::Mat final;

        std::vector< std::vector <cv::Point> > contours; // Vector for storing contour

        std::vector< cv::Vec4i > hierarchy;
        cv::findContours( canny_output, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
        if(contours.size()==0)
        {
            cv::threshold(S,S,20,255,0);
            cv::Canny(S,canny_output,lowThreshold,lowThreshold*ratio,kernel_size);
            cv::findContours( canny_output, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
        }
        cv::imshow("Canny Output", canny_output);
        std::vector< std::vector<cv::Point> > hull(contours.size());
        for(int i = 0; i < contours.size(); i++)
            cv::convexHull(cv::Mat(contours[i]), hull[i], false);
        double maxArea=0;
        double maxAreaContourId = 0;
        std::cout<<contours.size()<<std::endl;
        for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
        //for( int i = 0; i< hull.size(); i++ )
        {
            double newArea=cv::contourArea(contours[i]);
            double convexArea=cv::contourArea(hull[i]);
            if(convexArea/newArea>1.2)
                continue;
            if (newArea > maxArea) {
                maxArea = newArea;
                maxAreaContourId = i;
            } 
        }

        final.create( S.size(), S.type() );
        final = cv::Scalar::all(0);
        cv::drawContours(final, contours, maxAreaContourId, 255, -1);
        // cv::imshow("final",final);

        std::vector<cv::Vec4i> linesP; // will hold the results of the detection
        cv::HoughLinesP(final, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
        
        if(linesP.size()==0)
        {
            cv::HoughLinesP(final, linesP, 1, CV_PI/180, 10, 10, 10 ); 
        }
        float xtot=0,ytot=0,vec_len=linesP.size();
        for( size_t i = 0; i < vec_len; i++ )
        {
            xtot+=(linesP[i])[0]+(linesP[i])[2];
            ytot+=(linesP[i])[1]+(linesP[i])[3];
        }
        std::cout<<"x : "<<xtot<<std::endl;
        std::cout<<"y : "<<ytot<<std::endl;
        std::cout<<"len : "<<vec_len<<std::endl;
        // std::cout<<canny_output.type()<<std::endl;        
        xtot=xtot/(2*vec_len);
        ytot=ytot/(2*vec_len);
        // xtot+=5;
        std::cout<<xtot<<std::endl;
        cv::circle( canny_output,cv::Point(xtot,ytot), 5,  cv::Scalar(255), 2, 8, 0 );
        cv::imshow("Contour", final);
        cv::imshow("Edges with center", canny_output);
        cv::waitKey(3);
   
       // Output modified video stream
       cv_bridge::CvImage out_msg;
       out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1; // Or whatever    sensor_msgs::image_encodings::TYPE_32FC1;
      //  out_msg.encoding=cv_ptr->encoding;
       out_msg.image    = canny_output; // Your cv::Mat

   
       image_pub_.publish(out_msg.toImageMsg());
     
     }
   };
   
   
   int main(int argc, char** argv)
   {
        ros::init(argc, argv, "image_converter");
        ImageConverter ic;
        ros::spin();
   
        return 0;
   }


// #include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

// sensor_msgs::Image info;
// cv_bridge::CvImagePtr cv_ptr;

// cv_bridge::CvImage out_msg;
// static const std::string OPENCV_WINDOW = "Image window";

// void cb(const sensor_msgs::ImageConstPtr& msg)
// {

//     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

//     if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//          cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

//     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//     cv::waitKey(3);


//     out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
//     out_msg.encoding = cv_ptr->encoding; // Or whatever    sensor_msgs::image_encodings::TYPE_32FC1;
//     out_msg.image    = cv_ptr->image; // Your cv::Mat
// }

// int main(int argc , char ** argv)
// {
//     ros::init(argc,argv,"image_converter");
//     ros::NodeHandle nh;
//     image_transport::ImageTransport it(nh);
//     image_transport::Subscriber sub=it.subscribe("/iris/camera_red_iris/image_raw", 1,cb);
//     image_transport::Publisher feed_pub = it.advertise("/image_converter/output_video", 1);


       
//      feed_pub.publish(out_msg.toImageMsg());
//     // feed_pub.publish(cv_ptr->toImageMsg());
//     ros::spin();

//     return 0;
// }


////////////////////////////depth
#include <ros/ros.h>
  #include <opencv2/imgproc/imgproc.hpp>
  #include <opencv2/highgui/highgui.hpp>
  #include <iostream>
  #include <string>
  #include <numeric>


  double Depth(cv::Mat final){
    std::vector<cv::Vec4i> linesP; // will hold the results of the detection
    cv::HoughLinesP(final, linesP, 1, CV_PI/180, 200, 50, 10 ); // runs the actual detection
    const double ref_dist=1;
    const double ref_size=58.41;
     
    if(linesP.size()==0)
    {
        cv::HoughLinesP(final, linesP, 1, CV_PI/180, 10, 10, 10 ); 
    }
    int vec_len=linesP.size();
    double depth=0;
    for( size_t i = 0; i < vec_len; i++ )
    {   
        double x1=linesP[i][0];
        double y1=linesP[i][1];
        double x2=linesP[i][2];
        double y2=linesP[i][3];
        double d=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
        d=(ref_size/d)*ref_dist;
        depth+=d;
    }
    
    depth=depth/(vec_len);
    depth=depth*10;
    // std::cout<<"depth:"<<depth<<std::endl;

    return depth;
  }