#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/ximgproc.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>

using namespace cv;
using namespace std;
using namespace cv::ximgproc;

enum class CameraSide { LEFT, RIGHT };
int min_disparity = 0;
int num_disparities = 112;
int sad_window_size = 16;
int p1 = 240;
int p2 = 960;
int disp_12_max_diff = 1;
int pre_filter_cap = 1;
int uniqueness_ratio = 10;
int speckle_window_size = 100;
int speckle_range = 32;

// int min_disparity = 0;
// int num_disparities = 64;
// int sad_window_size = 3;
// int p1 = 240;
// int p2 = 960;
// int disp_12_max_diff = -1;
// int pre_filter_cap = 1;
// int uniqueness_ratio = 10;
// int speckle_window_size = 1;
// int speckle_range = 32;

int mode = cv::StereoSGBM::MODE_SGBM_3WAY;
double focal_length = 400;;
double baseline = 0.124463;
double stereo_cx = 519.5;
double stereo_cy = 399.5;

void init_rectification_map(string param_file_path);
void synched_img_callback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right);
void undistort_rectify_image(Mat& src, Mat& dst, const CameraSide& side);
void elaborateImages(const std_msgs::Header &header_msg, Mat& src, Mat& dst);
void computePointcloud(const cv::Mat &input_disparity, sensor_msgs::PointCloud2 &pointcloud);

//////////////////////////////////////////////////
// Declare all the calibration matrices as Mat variables.s
//////////////////////////////////////////////////
Mat lmapx, lmapy, rmapx, rmapy, Q;

image_transport::Publisher pub_img_rect_left, pub_img_rect_right, pub_disparity;

sensor_msgs::CameraInfo output_camera_info_left, output_camera_info_right;

ros::Publisher left_camera_info_output_pub, right_camera_info_output_pub, pub_pointcloud;

std::string output_frame_id = "camera_fisheye1_optical_frame";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_fisheye_undistort");

  ros::NodeHandle nh("~");

  string param_file_path;
  if(nh.getParam("param_file_path", param_file_path))
  {
    ROS_WARN("Using parameter file: %s", param_file_path.c_str());
  }
  else
  {
    ROS_ERROR("Failed to get param file path. Please check and try again.");
    ros::shutdown();
    return 0;
  }

  // Read the input parameters and perform initialization
  init_rectification_map(param_file_path);

  // The raw stereo images should be published as type sensor_msgs/Image
  image_transport::ImageTransport it(nh);
  message_filters::Subscriber<sensor_msgs::Image> sub_img_left(nh, "/camera/fisheye1/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_img_right(nh, "/camera/fisheye2/image_raw", 1);
  
  // Having time synced stereo images might be important for other purposes, say generating accurate disparity maps. 
  // To sync the left and right image messages by their header time stamps, ApproximateTime is used.
  // More info here: http://wiki.ros.org/message_filters#Time_Synchronizer
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_img_left, sub_img_right);
  sync.registerCallback(boost::bind(&synched_img_callback, _1, _2));

  // The output data include rectified images and their corresponding camera info
  pub_img_rect_left  = it.advertise("/camera/fisheye1/rect/image", 1);
  pub_img_rect_right = it.advertise("/camera/fisheye2/rect/image", 1);
  pub_disparity = it.advertise("/disparity", 10);
  pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud2", 10);

  // pub_disparity = it.advertise("/camera/disparity", 10);

  left_camera_info_output_pub  = nh.advertise<sensor_msgs::CameraInfo>("/camera/fisheye1/rect/camera_info", 1);
  right_camera_info_output_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/fisheye2/rect/camera_info", 1);

  // Processing start
  ros::spin();
}


//////////////////////////////////////////////////
// This function computes all the projection matrices and the rectification transformations 
// using the stereoRectify and initUndistortRectifyMap functions respectively.
// See documentation for stereoRectify: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereorectify
//////////////////////////////////////////////////
void init_rectification_map(string param_file_path) 
{
  Mat P1, P2, nP1, nP2;
  Mat R1, R2, K1, K2, D1, D2, R;
  Vec3d T;
  Vec2d size_input, size_output;

  FileStorage param_file = FileStorage(param_file_path, FileStorage::READ);
  param_file["K1"] >> K1;
  param_file["D1"] >> D1;
  param_file["K2"] >> K2;
  param_file["D2"] >> D2;
  param_file["R"]  >> R;
  param_file["T"]  >> T;
  param_file["input"]  >> size_input;
  param_file["output"] >> size_output;

  // The resolution of the input images used for stereo calibration.
  Size input_img_size(size_input[0], size_input[1]);

  // The resolution of the output rectified images. Lower resolution images require less computation time.
  Size output_img_size(size_output[0], size_output[1]);
  double alpha = 0.0;

  stereoRectify(K1, D1, K2, D2, 
                input_img_size, 
                R, T, 
                R1, R2, P1, P2, 
                Q,
                1024, 
                alpha, 
                output_img_size);
  nP1 = P1;
  double A1 = P1.at<double>(0,0);
  double A2 = P1.at<double>(1,1);
  nP1.at<double>(0,0) = A1/2.0;
  nP1.at<double>(1,1) = A2/2.0;

  nP2 = P2;
  double B1 = P2.at<double>(0,0);
  double B2 = P2.at<double>(1,1);
  nP2.at<double>(0,0) = B1/2.0;
  nP2.at<double>(1,1) = B2/2.0;

  fisheye::initUndistortRectifyMap(K1, D1, R1, nP1, output_img_size, CV_32FC1, lmapx, lmapy);
  fisheye::initUndistortRectifyMap(K2, D2, R2, nP2, output_img_size, CV_32FC1, rmapx, rmapy);

  // focal_length = nP1.at<double>(0,0);
  // baseline = std::abs(T[0]);
  // Copy the parameters for rectified images to the camera_info messages
  output_camera_info_left.width   = size_output[0];
  output_camera_info_left.height  = size_output[1];
  output_camera_info_left.D       = vector<double>(5, 0);

  output_camera_info_right.width  = size_output[0];
  output_camera_info_right.height = size_output[1];
  output_camera_info_right.D      = vector<double>(5, 0);

  for (int i = 0; i < 9; i++)
  {
    output_camera_info_left.K[i]  = K1.at<double>(i);
    output_camera_info_right.K[i] = K2.at<double>(i);
    output_camera_info_left.R[i]  = R1.at<double>(i);
    output_camera_info_right.R[i] = R2.at<double>(i);
  }  
  for (int i = 0; i < 12; i++)
  {
    output_camera_info_left.P[i]  = P1.at<double>(i);
    output_camera_info_right.P[i] = P2.at<double>(i);
  }
  
  ROS_INFO("Initialization complete. Publishing rectified images and camera_info when raw images arrive...");
}


//////////////////////////////////////////////////
// This callback function takes a pair of raw stereo images as inputs, 
// then undistorts and rectifies the images using the undistort_rectify_image function 
// defined above and publishes on the rectified image topic using pub_img_left/right.
//////////////////////////////////////////////////
void synched_img_callback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right)
{
    Mat tmp_left  = cv_bridge::toCvShare(msg_left, "mono8")->image;
    Mat tmp_right = cv_bridge::toCvShare(msg_right, "mono8")->image;

    Mat dst_left, dst_right;

    undistort_rectify_image(tmp_left, dst_left, CameraSide::LEFT);
    undistort_rectify_image(tmp_right, dst_right, CameraSide::RIGHT);

    sensor_msgs::ImagePtr rect_img_left  = cv_bridge::CvImage(msg_left->header, "mono8", dst_left).toImageMsg();
    sensor_msgs::ImagePtr rect_img_right = cv_bridge::CvImage(msg_right->header, "mono8", dst_right).toImageMsg();

    pub_img_rect_left.publish(rect_img_left);
    pub_img_rect_right.publish(rect_img_right);

    std_msgs::Header header = msg_left->header;

    elaborateImages(header, dst_left, dst_right);
    output_camera_info_left.header = header;
    output_camera_info_left.header.frame_id = output_frame_id;
    output_camera_info_right.header = header;
    output_camera_info_right.header.frame_id = output_frame_id;

    left_camera_info_output_pub.publish(output_camera_info_left);
    right_camera_info_output_pub.publish(output_camera_info_right);
}

//////////////////////////////////////////////////
// This function undistorts and rectifies the src image into dst. 
// The homographic mappings lmapx, lmapy, rmapx, and rmapy are found from OpenCV’s initUndistortRectifyMap function.
//////////////////////////////////////////////////
void undistort_rectify_image(Mat& src, Mat& dst, const CameraSide& side)
{
  if (side == CameraSide::LEFT) 
  {
    remap(src, dst, lmapx, lmapy, cv::INTER_LINEAR);
  } 
  else 
  {
    remap(src, dst, rmapx, rmapy, cv::INTER_LINEAR);
  }
}



void elaborateImages(const std_msgs::Header &header_msg, Mat& dst_left, Mat& dst_right)
{
    // start depth image operations
    cv::Mat left_disp, left_disp8u, right_disp, right_disp8u, Q, image3D, disp_vis, disp_color;
    cv::Mat filtered_disp, filtered_disp8u, disp_vis1;
    Ptr<DisparityWLSFilter> wls_filter;

    cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(
      min_disparity,
      num_disparities,
      sad_window_size,
      p1,
      p2,
      disp_12_max_diff,
      pre_filter_cap,
      uniqueness_ratio,
      speckle_window_size,
      speckle_range,
      mode);

    // get disparity map
    left_matcher->compute(dst_left, dst_right, left_disp);
    Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
    right_matcher->compute(dst_right, dst_left, right_disp);

    // 3D image
    // Q = 
    // reprojectImageTo3D(left_disp, pointcloud, Q, false);

    // crealte wls filter for disparity map
    wls_filter = createDisparityWLSFilter(left_matcher);
    wls_filter->setLambda(8000.0);
    wls_filter->setSigmaColor(1.5);
    wls_filter->filter(left_disp,dst_left,filtered_disp,right_disp);

    // normalize disparity map
    medianBlur(filtered_disp, filtered_disp, 3);
    cv::normalize(filtered_disp, filtered_disp8u, 0, 255, cv::NORM_MINMAX, CV_8U);
    medianBlur(left_disp, left_disp, 3);
    cv::normalize(left_disp, left_disp8u, 0, 255, cv::NORM_MINMAX, CV_8U);

    // disp_vis = (baseline*focal_length)/filtered_disp;
    // disp_vis = 255*(filtered_disp - min_disparity)/ num_disparities;
    // convertScaleAbs(disp_vis, disp_vis1, 1);
    // applyColorMap(disp_vis1, disp_color, 2);

    // publish disparity image
    sensor_msgs::ImagePtr out_disparity_msg;
    out_disparity_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", filtered_disp8u).toImageMsg();
    pub_disparity.publish(out_disparity_msg);

    // publish point could image
    // build pointcloud
    sensor_msgs::PointCloud2 pointcloud_msg;
    computePointcloud(left_disp, pointcloud_msg);
    // cv::reprojectImageTo3D(filtered_disp, image3D, Q, true);
    // publish pointcloud
    pointcloud_msg.header.stamp = header_msg.stamp;
    pointcloud_msg.header.frame_id = output_frame_id;

    if (pub_pointcloud.getNumSubscribers() > 0)
    {
    pub_pointcloud.publish(pointcloud_msg);
    }
}



void computePointcloud(const cv::Mat &input_disparity, sensor_msgs::PointCloud2 &pointcloud)
{
    // build pointcloud
    int side_bound = sad_window_size / 2;
    sensor_msgs::PointCloud2Modifier modifier(pointcloud);
    pointcloud.width = input_disparity.cols - side_bound;
    pointcloud.height = input_disparity.rows - side_bound;
    modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> pointcloud_x(pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<float> pointcloud_y(pointcloud, "y");
    sensor_msgs::PointCloud2Iterator<float> pointcloud_z(pointcloud, "z");

    for (int y_pixels = side_bound; y_pixels < input_disparity.rows - side_bound;
          ++y_pixels)
    {
        for (int x_pixels = side_bound + min_disparity + num_disparities;
              x_pixels < input_disparity.cols - side_bound; ++x_pixels)
        {
            const int16_t &input_value = input_disparity.at<int16_t>(y_pixels, x_pixels);
            double disparity_value;
            disparity_value = static_cast<double>(input_value);

            // double min, max;
            // cv::minMaxLoc(input_disparity, &min, &max);
            // std::cout << "min: " << min << " max: " << max << std::endl;
            // min: -16 max: 1008

            // the 16* is needed as opencv stores disparity maps as 16 * the true
            // values
            if (disparity_value > 50 && disparity_value < 800)
            {
                *pointcloud_z = (16 * focal_length * baseline) / disparity_value;
                *pointcloud_x = *pointcloud_z * (x_pixels - stereo_cx) / focal_length;
                *pointcloud_y = *pointcloud_z * (y_pixels - stereo_cy) / focal_length;
            }

            ++pointcloud_x;
            ++pointcloud_y;
            ++pointcloud_z;
        }
    }
}