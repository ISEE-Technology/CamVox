#include "Calibrating.h"
#include "Tracking.h"
#include "LocalMapping.h"
#include "LoopClosing.h"

//#define debug
#define pi (3.14159267)
#define rad2deg(x) (x) * 180 / pi
#define deg2rad(x) (x) * pi / 180

bool color2dDepth = false;


const string RGBPath = "./camvox/calibration/calibration.bmp";
const string PcdPath = "./camvox/calibration/calibration.pcd"; 
const string projectionType = "depth";
bool isEnhanceImg = false;
bool isFillImg = true;

namespace Camvox
{
  /********************************************Calibration**************************************************************************/
  Calibrating::Calibrating(string _strSettingPath) : mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbCalibrating(false), mbOptimizing(false),cumulative_flag(false)
  {
    // initialize parameters
    depth_edge_threshold_ = 80;
    best_r_ = 0;
    best_p_ = 0;
    best_y_ = 0;
    optimize_type = 0;
    loadParams(_strSettingPath);
  
  }
  

  void Calibrating::initialize(string _RGB_path, string _Pcd_path, string _Projection_type, bool _isEnhanceImg, bool _isFillImg)
  {
    loadPcd(_Pcd_path);
    is_enhancement_ = _isEnhanceImg;
    is_fillImg_ = _isFillImg;
    Eigen::Matrix3d rotation_matrix;

    // our initial parameters
    rotation_matrix << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    Eigen::Vector3d euler_angle(deg2rad(-0.45), deg2rad(-0.4), deg2rad(-0.3));
    Eigen::Matrix3d rotation_matrix_adjust;

    // Rotate counterclockwise about the axis (rad)
    rotation_matrix_adjust = Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitX());
    cout << "euler_angle:" << euler_angle << endl;
    Vector6d extrinsic_params;
    Eigen::Matrix3d test_rotation_matrix = rotation_matrix * rotation_matrix_adjust;
    cout << "use rotation matrix =\n"
         << test_rotation_matrix << endl;
    Eigen::Vector3d test_angle = test_rotation_matrix.eulerAngles(2, 1, 0);
    extrinsic_params << test_angle[0], test_angle[1], test_angle[2], T_[0], T_[1], T_[2];

    // read Camera image
    origin_RGB_ = imread(_RGB_path, CV_LOAD_IMAGE_UNCHANGED);
    imRGB_ = imread(_RGB_path, CV_LOAD_IMAGE_UNCHANGED);
    // intercept RGB image
    rect_.x = 40;
    rect_.y = 40;
    rect_.width = 1300;
    rect_.height = 500;
    imRGB_ = imRGB_(rect_);

    // RGB2GRAY
    cvtColor(imRGB_, imGray_, CV_BGR2GRAY);
    // histogram equalization
    if (is_enhancement_)
    {
      equalizeHist(imGray_, imGray_);
    }

    // projection both use Intensity and Depth image
    if (_Projection_type == "intensity")
    {
      projection_type_ = INTENSITY;
    }
    else if (_Projection_type == "depth")
    {
      projection_type_ = DEPTH;
    }
    else
    {
      projection_type_ = BOTH;
    }
    Projection(projection_type_, extrinsic_params);

    //EdgeDetection canny
    rgb_canny_threshold_ = 30;
    depth_canny_threshold_ = 20;
    intensity_canny_threshold_ = 30;
    EdgeDetction(projection_type_, rgb_canny_threshold_, depth_canny_threshold_, intensity_canny_threshold_);

#ifdef debug
    color3d(extrinsic_params, 1, "./camvox/debug/test.pcdl");  
    cv::imshow("imGRAY_", imGray_);
    cv::imshow("RGB_canny", rgb_canny_);
    cv::waitKey();
    if (projection_type_ == INTENSITY)
    {
      cv::imshow("imIntensity_", imIntensity8_);
      cv::imshow("Intensity canny", intensity_canny_);
    }
    else if (projection_type_ == DEPTH)
    {
      cv::imshow("imDepth_", imdepth8_);
      cv::imshow("Depth canny", depth_canny_);
    }
    else
    {
      cv::imshow("imDepth_", imdepth8_);
      cv::imshow("Depth canny", depth_canny_);
      cv::imshow("imIntensity_", imIntensity8_);
      cv::imshow("Intensity canny", intensity_canny_);
    }
    cv::waitKey();
#endif

    // Edge filter and build Point cloud
    BuildRgbCloud();
    BuildLidarCloud();
    initRgbUseClouds();
    rgb_use_cloud_xyzrgb_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    lidar_use_cloud_xyzrgb_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    convertToRgbCloud(rgb_cloud_xyz_, rgb_cloud_xyzrgb_, Eigen::Vector3i(255, 0, 0));
    convertToRgbCloud(lidar_cloud_xyz_, lidar_cloud_xyzrgb_, Eigen::Vector3i(0, 0, 255));
    float init_distance = calcMeanDistance(rgb_use_cloud_, lidar_use_cloud_);
    cout << "init distance:" << init_distance << endl;
    convertToRgbCloud(rgb_use_cloud_, rgb_use_cloud_xyzrgb_, Eigen::Vector3i(255, 0, 0));
    convertToRgbCloud(lidar_use_cloud_, lidar_use_cloud_xyzrgb_, Eigen::Vector3i(0, 0, 255));

#ifdef debug
    std::string info = "best rotation, cost" + std::to_string(init_distance);
    if (projection_type_ == DEPTH)
    {
      color2d(true, info);
    }
    else
    {
      color2d(false, info);
    }
    showOrgPointClouds("init viewer");
    showClouds(rgb_use_cloud_xyzrgb_, lidar_use_cloud_xyzrgb_);
#endif

  }



  /***************************************************loadPcd*******************************************************************/
  bool Calibrating::loadPcd(const std::string &pcd_file)
  {
    horizon_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile(pcd_file, *horizon_cloud_) != -1)
    {
      std::cout << "load pcd file successfully! size:" << horizon_cloud_->points.size() << std::endl;
      return true;
    }
    else
    {
      std::cout << "failed to load pcd file" << std::endl;
      return false;
    }
  }

  /****************************************loadParams******************************************************************************/
  bool Calibrating::loadParams(const std::string &filename)
  {
    cv::FileStorage fSettings(filename, cv::FileStorage::READ);
    if (!fSettings.isOpened())
    {
      cerr << "Failed to open settings file at: " << filename << endl;
      exit(-1);
    }
    fx_ = fSettings["Camera.fx"];
    fy_ = fSettings["Camera.fy"];
    cx_ = fSettings["Camera.cx"];
    cy_ = fSettings["Camera.cy"];
    k1_ = fSettings["Camera.k1"];
    k2_ = fSettings["Camera.k2"];
    p1_ = fSettings["Camera.P1"];
    p2_ = fSettings["Camera.P2"];
    k3_ = fSettings["Camera.k3"];
    bf = fSettings["Camera.bf"];
    width_ = fSettings["Camera.width"];
    height_ = fSettings["Camera.height"];
    rect_.x = fSettings["Rect.x"];
    rect_.y = fSettings["Rect.y"];
    rect_.width = fSettings["Rect.width"];
    rect_.height = fSettings["Rect.height"];

#ifdef debug
    cout << "fx:" << fx_ << " fy:" << fy_ << " cx:" << cx_ << " cy:" << cy_ << endl;
    cout << "k1:" << k1_ << " k2:" << k2_ << " p1:" << p1_ << " p2:" << p2_ << " k3:" << k3_ << endl;
#endif
    mDepthMapFactor_ = fSettings["DepthMapFactor"];
    mDepthMapFactor_ = 1 / mDepthMapFactor_;
  }

  /***********************************************Projection***********************************************************************/
  void Calibrating::Projection(const ProjectionType projection_type, const Vector6d &extrinsic_params)
  {
    if (projection_type == INTENSITY)
    {
      imIntensity_ = convertByIntensity(extrinsic_params);
      imIntensity_ = imIntensity_(rect_);
      imIntensity_.convertTo(imIntensity8_, CV_8U, 1.0 / 256);
      if (is_enhancement_)
      {
        equalizeHist(imIntensity8_, imIntensity8_);
      }
    }
    else if (projection_type == DEPTH)
    {
      imdepth_ = convertByDepth(extrinsic_params, is_fillImg_);
      imdepth_ = imdepth_(rect_);
      imdepth_.convertTo(imdepth8_, CV_8U, 1.0 / 256);
      /********************DEBUG************************/
      cout << "DEPTH" << endl;
      /*************************************************/
      if (is_enhancement_)
        equalizeHist(imdepth8_, imdepth8_);
    }
    else
    {
      imIntensity_ = convertByIntensity(extrinsic_params);
      imIntensity_ = imIntensity_(rect_);
      imIntensity_.convertTo(imIntensity8_, CV_8U, 1.0 / 256);
      if (is_enhancement_)
        equalizeHist(imIntensity8_, imIntensity8_);
      imdepth_ = convertByDepth(extrinsic_params, is_fillImg_);
      imdepth_ = imdepth_(rect_);
      imdepth_.convertTo(imdepth8_, CV_8U, 1.0 / 256);
      if (is_enhancement_)
        equalizeHist(imdepth8_, imdepth8_);
    }
  }

  /*****************************************************convertToRgbCloud*****************************************************************/
  void Calibrating::convertToRgbCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &xyz_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &xyzrgb_cloud, Eigen::Vector3i rgb)
  {
    xyzrgb_cloud->points.clear();
    for (size_t i = 0; i < xyz_cloud->points.size(); i++)
    {
      pcl::PointXYZRGB p;
      p.x = xyz_cloud->points[i].x;
      p.y = xyz_cloud->points[i].y;
      p.z = xyz_cloud->points[i].z;
      p.r = rgb[0];
      p.g = rgb[1];
      p.b = rgb[2];
      p.a = 255;
      xyzrgb_cloud->points.push_back(p);
    }
    xyzrgb_cloud->width = xyzrgb_cloud->points.size();
    xyzrgb_cloud->height = 1;
  }

  /***************************************************convertByIntensity*******************************************************************/
  Mat Calibrating::convertByIntensity(const Vector6d &calib_params)
  {
    int max_intensity = 100;
    std::vector<cv::Point3f> pts_3d;
    Eigen::AngleAxisd rotation_vector3;
    rotation_vector3 = Eigen::AngleAxisd(calib_params[0], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(calib_params[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(calib_params[2], Eigen::Vector3d::UnitX());
    for (size_t i = 0; i < horizon_cloud_->size(); ++i)
    {
      pcl::PointXYZI point_3d = horizon_cloud_->points[i];
      pts_3d.emplace_back(cv::Point3f(point_3d.x, point_3d.y, point_3d.z));
    }
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
    cv::Mat distortion_coeff = (cv::Mat_<double>(1, 5) << k1_, k2_, p1_, p2_, k3_);
    cv::Mat r_vec = (cv::Mat_<double>(3, 1) << rotation_vector3.angle() * rotation_vector3.axis().transpose()[0], rotation_vector3.angle() * rotation_vector3.axis().transpose()[1], rotation_vector3.angle() * rotation_vector3.axis().transpose()[2]);
    cv::Mat t_vec = (cv::Mat_<double>(3, 1) << calib_params[3], calib_params[4], calib_params[5]);
    // project 3d-points into image view
    std::vector<cv::Point2f> pts_2d;
    cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff, pts_2d);
    // cv::calcBackProject()
    cv::Mat image_project = cv::Mat::zeros(height_, width_, CV_16UC1);
    int image_rows = image_project.rows;
    int image_cols = image_project.cols;
    for (size_t i = 0; i < pts_2d.size(); ++i)
    {
      cv::Point2f point_2d = pts_2d[i];
      if (point_2d.x <= 0 || point_2d.x >= image_cols || point_2d.y <= 0 || point_2d.y >= image_rows)
      {
        continue;
      }
      else
      {
        float intensity = horizon_cloud_->points[i].intensity;
        if (intensity > max_intensity)
        {
          intensity = 65535;
        }
        else
        {
          intensity = intensity / max_intensity * 65535;
        }
        image_project.at<ushort>(point_2d.y, point_2d.x) = intensity;
      }
    }
    Mat filed_img = image_project.clone();
    for (size_t x = 0; x < image_project.cols; x++)
    {
      for (size_t y = 0; y < image_project.rows; y++)
      {
        if (image_project.at<ushort>(y, x) == 0)
        {
          std::vector<ushort> temp_intensity;
          for (int x_inc = 0; x_inc < 4; x_inc++)
          {
            for (int y_inc = 0; y_inc < 4; y_inc++)
            {
              int x_near = x + pow(-1, x_inc) * int((x_inc + 2) / 2);
              int y_near = y + pow(-1, y_inc) * int((y_inc + 2) / 2);
              if (x_near > 0 && y_near > 0 && image_project.at<ushort>(y_near, x_near) != 0)
              {
                temp_intensity.push_back(image_project.at<ushort>(y_near, x_near));
              }
            }
            if (temp_intensity.size() != 0)
            {
              float mean_intensity = 0;
              for (size_t i = 0; i < temp_intensity.size(); i++)
              {
                mean_intensity += temp_intensity[i];
              }
              filed_img.at<ushort>(y, x) = mean_intensity / temp_intensity.size();
            }
          }
        }
      }
    }
#ifdef debug

    cv::imshow("project by intensiy", filed_img);
    cv::imwrite("./camvox/debug/intensity.jpg", filed_img);
    cv::waitKey();

#endif
    return filed_img;
  }

  /*************************************************convertByDepth*********************************************************************/
  Mat Calibrating::convertByDepth(const Vector6d &calib_params, const bool is_fillImg)
  {
    std::vector<cv::Point3f> pts_3d;
    Eigen::AngleAxisd rotation_vector3;
    rotation_vector3 = Eigen::AngleAxisd(calib_params[0], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(calib_params[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(calib_params[2], Eigen::Vector3d::UnitX());
    double mean_distance = 0;
    for (size_t i = 0; i < horizon_cloud_->size(); ++i)
    {
      pcl::PointXYZI point_3d = horizon_cloud_->points[i];
      pts_3d.emplace_back(cv::Point3f(point_3d.x, point_3d.y, point_3d.z));
      mean_distance += sqrt(pow(point_3d.x, 2) + pow(point_3d.y, 2) + pow(point_3d.z, 2)) * (1 / horizon_cloud_->size());
    }

    float max_depth = 80;
    float min_depth = 1;

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
    cv::Mat distortion_coeff = (cv::Mat_<double>(1, 5) << k1_, k2_, p1_, p2_, k3_);
    cv::Mat r_vec = (cv::Mat_<double>(3, 1) << rotation_vector3.angle() * rotation_vector3.axis().transpose()[0], rotation_vector3.angle() * rotation_vector3.axis().transpose()[1], rotation_vector3.angle() * rotation_vector3.axis().transpose()[2]);
    cv::Mat t_vec = (cv::Mat_<double>(3, 1) << calib_params[3], calib_params[4], calib_params[5]);
    // project 3d-points into image view
    std::vector<cv::Point2f> pts_2d;
    cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff, pts_2d);
    cv::Mat image_project = cv::Mat::zeros(height_, width_, CV_16UC1);
    int image_rows = image_project.rows;
    int image_cols = image_project.cols;
    for (size_t i = 0; i < pts_2d.size(); ++i)
    {
      cv::Point2f point_2d = pts_2d[i];
      if (point_2d.x <= 0 || point_2d.x >= image_cols || point_2d.y <= 0 || point_2d.y >= image_rows)
      {
        continue;
      }
      else
      {
        float depth = sqrt(pow(horizon_cloud_->points[i].x, 2) + pow(horizon_cloud_->points[i].y, 2) + pow(horizon_cloud_->points[i].z, 2));
        if (depth > max_depth)
        {
          depth = 65535;
        }
        else
        {
          depth = (depth - min_depth) / max_depth * 65535;
          // cout << "depth:" << depth << endl;
        }
        image_project.at<ushort>(point_2d.y, point_2d.x) = depth;
      }
    }

    imdepth_true_ = image_project.clone();
    Mat filed_img = image_project.clone();
    if (is_fillImg)
    {

      for (size_t x = 0; x < image_project.cols; x++)
      {
        for (size_t y = 0; y < image_project.rows; y++)
        {
          if (image_project.at<ushort>(y, x) == 0)
          {
            std::vector<ushort> temp_depth;
            for (int x_inc = 0; x_inc < 4; x_inc++)
            {
              for (int y_inc = 0; y_inc < 4; y_inc++)
              {
                int x_near = x + pow(-1, x_inc) * int((x_inc + 2) / 2);
                int y_near = y + pow(-1, y_inc) * int((y_inc + 2) / 2);
                if (x_near > 0 && y_near > 0 && image_project.at<ushort>(y_near, x_near) != 0)
                {
                  temp_depth.push_back(image_project.at<ushort>(y_near, x_near));
                }
              }
              if (temp_depth.size() > 1)
              {
                float mean_depth = 0;
                for (size_t i = 0; i < temp_depth.size(); i++)
                {
                  mean_depth += temp_depth[i];
                }
                filed_img.at<ushort>(y, x) = mean_depth / temp_depth.size();
              }
            }
          }
        }
      }
    }
#ifdef debug

    cv::imshow("project by depth(org)", image_project);
    cv::imshow("project by depth(filled)", filed_img);
    cv::imwrite("./camvox/debug/depth.jpg", image_project);
    cv::waitKey();
#endif
    return filed_img;
  }

  /*************************************************EdgeDetction*********************************************************************/
  void Calibrating::EdgeDetction(const ProjectionType projection_type, const int rgb_threshold, const int depth_threshold, const int intensity_threshold)
  {
    // Canny
    Mat im_gray, im_intensity, im_depth;
    Mat imdepth_Canny, Intensity_Canny, RGB_Canny;
    int gaussian_size = 5;

    GaussianBlur(imGray_, im_gray, cv::Size(gaussian_size, gaussian_size), 0, 0);

    Canny(im_gray, rgb_canny_, rgb_threshold, rgb_threshold * 3, 3, true);
    findContours(rgb_canny_, contours_RGB_, hierarchy_RGB_, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point());

    if (projection_type == INTENSITY)
    {
      blur(imIntensity8_, im_intensity, Size(5, 5), Point(-1, -1));
      Canny(im_intensity, intensity_canny_, intensity_threshold, intensity_threshold * 3, 3, true);
      findContours(intensity_canny_, contours_intensity_, hierarchy_intensity_, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point());
    }
    else if (projection_type == DEPTH)
    {
      GaussianBlur(imdepth8_, im_depth, cv::Size(gaussian_size, gaussian_size), 0, 0);
      Canny(im_depth, depth_canny_, depth_threshold, depth_threshold * 3, 3, true);
      findContours(depth_canny_, contours_imdepth_, hierarchy_imdepth_, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point());
    }
    else
    {
      blur(imIntensity8_, im_intensity, Size(5, 5), Point(-1, -1));
      Canny(im_intensity, intensity_canny_, intensity_threshold, intensity_threshold * 3, 3, true);
      findContours(intensity_canny_, contours_intensity_, hierarchy_intensity_, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point());
      GaussianBlur(imdepth8_, im_depth, cv::Size(gaussian_size, gaussian_size), 0, 0);
      Canny(im_depth, depth_canny_, depth_threshold, depth_threshold * 3, 3, true);
      findContours(depth_canny_, contours_imdepth_, hierarchy_imdepth_, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point());
    }
  }

  /***************************************************extractEdge*******************************************************************/
  void Calibrating::extractEdge(const bool is_filter, const Mat &img, const vector<vector<Point>> &contous, const int min_len, Mat &edge_img, pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_cloud)
  {
    // cv::imshow("before etract", img);
    // cv::waitKey();
    for (int i = 0; i < edge_img.rows; i++)
    {
      for (int j = 0; j < edge_img.cols; j++)
      {
        edge_img.at<uchar>(i, j) = 0;
      }
    }
    float radius_min = 5;
    float radius_max = 8;
    pcl::PointCloud<pcl::PointXYZ>::Ptr kd_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (is_filter)
    {
      for (size_t i = 0; i < contous.size(); i++)
      {
        if (contous[i].size() > min_len)
          for (size_t j = 0; j < contous[i].size(); j++)
          {
            pcl::PointXYZ p;
            p.x = contous[i][j].x;
            p.y = contous[i][j].y;
            p.z = 0;
            kd_input_cloud->points.push_back(p);
          }
      }
      kd_input_cloud->height = 1;
      kd_input_cloud->width = kd_input_cloud->points.size();
      pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
      if (kd_input_cloud->points.size() == 0)
        return;
      kdtree->setInputCloud(kd_input_cloud);
      for (int i = 0; i < edge_img.rows; i++)
      {
        for (int j = 0; j < edge_img.cols; j++)
        {
          pcl::PointXYZ searchPoint;
          searchPoint.x = j;
          searchPoint.y = i;
          searchPoint.z = 0;
          std::vector<int> pointIdxRadiusSearch;
          std::vector<float> pointRadiusSquaredDistance;
          std::vector<int> pointIdxRadiusSearchMax;
          std::vector<float> pointRadiusSquaredDistanceMax;
          if (img.at<uchar>(i, j) == 0)
          {
            if (kdtree->radiusSearch(searchPoint, radius_min, pointIdxRadiusSearch, pointRadiusSquaredDistance) <= 0)
            {
              if (kdtree->radiusSearch(searchPoint, radius_max, pointIdxRadiusSearchMax, pointRadiusSquaredDistanceMax) > 0)
              {
                for (size_t k = 0; k < pointIdxRadiusSearchMax.size(); k++)
                {
                  edge_img.at<uchar>(kd_input_cloud->points[pointIdxRadiusSearchMax[k]].y, kd_input_cloud->points[pointIdxRadiusSearchMax[k]].x) = 255;
                }
              }
            }
          }
        }
      }
      for (int i = 0; i < edge_img.rows; i++)
      {
        for (int j = 0; j < edge_img.cols; j++)
        {
          if (edge_img.at<uchar>(i, j) == 255)
          {
            pcl::PointXYZ p;
            p.x = j;
            p.y = -i;
            p.z = 0;
            edge_cloud->points.push_back(p);
          }
        }
      }
    }
    else
    {
      for (size_t i = 0; i < contous.size(); i++)
      {
        if (contous[i].size() > min_len)
          for (size_t j = 0; j < contous[i].size(); j++)
          {
            pcl::PointXYZ p;
            p.x = contous[i][j].x;
            p.y = -contous[i][j].y;
            p.z = 0;
            edge_cloud->points.push_back(p);
            edge_img.at<uchar>(-p.y, p.x) = 255;
          }
      }
    }

    edge_cloud->width = edge_cloud->points.size();
    edge_cloud->height = 1;

#ifdef debug

    if (is_filter)
    {
      cv::imshow("after extract ", edge_img);
      cv::waitKey();
    }

#endif
  }

  /*********************************************************BuildRgbCloud*************************************************************/
  void Calibrating::BuildRgbCloud()
  {
    bool is_use_filter = false;
    Mat color_edge_img = rgb_canny_;
    rgb_cloud_xyz_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    rgb_cloud_xyzrgb_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    extractEdge(is_use_filter, rgb_canny_, contours_RGB_, 50, color_edge_img, rgb_cloud_xyz_);
    convertToRgbCloud(rgb_cloud_xyz_, rgb_cloud_xyzrgb_, Eigen::Vector3i(255, 0, 0));
  }

  /*****************************************************BuildLidarCloud*****************************************************************/
  void Calibrating::BuildLidarCloud()
  {
    bool is_use_filter = false;
    lidar_cloud_xyz_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    lidar_cloud_xyzrgb_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr intensity_cloud_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr depth_cloud_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (projection_type_ == INTENSITY)
    {
      Mat intensity_edge_img = intensity_canny_;
      extractEdge(is_use_filter, intensity_canny_, contours_intensity_, 150, intensity_edge_img, intensity_cloud_xyz);

#ifdef debug
      cv::imshow("intensity edge", intensity_edge_img);
      cv::waitKey();
#endif
    }
    else if (projection_type_ == DEPTH)
    {
      Mat depth_edge_img = depth_canny_;
      extractEdge(is_use_filter, depth_canny_, contours_imdepth_, depth_edge_threshold_, depth_edge_img, depth_cloud_xyz);
#ifdef debug
      cv::imshow("depth edge", depth_edge_img);
      cv::waitKey();
#endif
    }
    else
    {
      Mat intensity_edge_img = intensity_canny_;
      extractEdge(is_use_filter, intensity_canny_, contours_intensity_, 150, intensity_edge_img, intensity_cloud_xyz);
      Mat depth_edge_img = depth_canny_;
      extractEdge(is_use_filter, depth_canny_, contours_imdepth_, depth_edge_threshold_, depth_edge_img, depth_cloud_xyz);
#ifdef debug
      cv::imshow("intensity edge", intensity_edge_img);
      cv::waitKey();
#endif
#ifdef debug
      cv::imshow("depth edge", depth_edge_img);
      cv::waitKey();
#endif
    }

    for (size_t i = 0; i < intensity_cloud_xyz->size(); i++)
    {
      lidar_cloud_xyz_->points.push_back(intensity_cloud_xyz->points[i]);
    }
    for (size_t i = 0; i < depth_cloud_xyz->size(); i++)
    {
      lidar_cloud_xyz_->points.push_back(depth_cloud_xyz->points[i]);
    }

    lidar_cloud_xyz_->width = lidar_cloud_xyz_->points.size();
    lidar_cloud_xyz_->height = 1;
    convertToRgbCloud(lidar_cloud_xyz_, lidar_cloud_xyzrgb_, Eigen::Vector3i(0, 0, 255));
  }

  /*************************************************costFunction*********************************************************************/
  float Calibrating::costFunction(const double *abc)
  {
    float distance_threshold = 15;
    int K = 2;
    Vector3d t;
    Vector3d eluer;
    int t2 = clock();
    if (optimize_type == 0)
    {
      eluer[0] = abc[0];
      eluer[1] = best_p_;
      eluer[2] = best_y_;
    }
    else if (optimize_type == 1)
    {
      eluer[0] = best_r_;
      eluer[1] = abc[0];
      eluer[2] = best_y_;
    }
    else if (optimize_type == 2)
    {
      eluer[0] = best_r_;
      eluer[1] = best_p_;
      eluer[2] = abc[0];
    }
    Vector6d extrinsic_params;
    Eigen::Matrix3d init_rotaion_matrix;
    Eigen::Matrix3d adjust_rotation_matrix;
    init_rotaion_matrix << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    adjust_rotation_matrix = Eigen::AngleAxisd(eluer[0], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(eluer[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(eluer[2], Eigen::Vector3d::UnitX());
    Eigen::Matrix3d true_rotation_matrix = init_rotaion_matrix * adjust_rotation_matrix;
    Eigen::Vector3d true_euler = true_rotation_matrix.eulerAngles(2, 1, 0);
    extrinsic_params << true_euler[0], true_euler[1], true_euler[2], T_[0], T_[1], T_[2];
    float distance = 1000;
    Projection(projection_type_, extrinsic_params);
    t2 = clock() - t2;
    rgb_canny_threshold_ = 30;
    depth_canny_threshold_ = 20;
    intensity_canny_threshold_ = 30;
    // Canny edge detect

    EdgeDetction(projection_type_, rgb_canny_threshold_, depth_canny_threshold_, intensity_canny_threshold_);
    // Edge filter and build Point cloud
    int t3 = clock();
    BuildRgbCloud();
    BuildLidarCloud();

    if (initRgbUseClouds())
    {
      int t1 = clock();
      distance = calcMeanDistance(rgb_use_cloud_, lidar_use_cloud_);
      // cout << "mean distance:" << distance << endl;
      convertToRgbCloud(rgb_use_cloud_, rgb_use_cloud_xyzrgb_, Eigen::Vector3i(255, 0, 0));
      convertToRgbCloud(lidar_use_cloud_, lidar_use_cloud_xyzrgb_, Eigen::Vector3i(0, 0, 255));
      // showClouds(rgb_use_cloud_xyzrgb_, lidar_use_cloud_xyzrgb_);
    }
    return distance;
  }

  /*****************************************************RpyCostFunction*****************************************************************/
  float Calibrating::RpyCostFunction(const Eigen::Vector3f &rpy)
  {
    Vector3d eluer;
    eluer[0] = rpy[0];
    eluer[1] = rpy[1];
    eluer[2] = rpy[2];
    Eigen::Vector3f T(-0.0221759, 0.0727962, 0.082984);
    Vector6d extrinsic_params;
    extrinsic_params << eluer[0], eluer[1], eluer[2], T[0], T[1], T[2];
    Projection(projection_type_, extrinsic_params);
    rgb_canny_threshold_ = 30;
    depth_canny_threshold_ = 20;
    intensity_canny_threshold_ = 30;
    // Canny edge detect
    EdgeDetction(projection_type_, rgb_canny_threshold_, depth_canny_threshold_, intensity_canny_threshold_);
    // Edge filter and build Point cloud
    BuildRgbCloud();
    BuildLidarCloud();
    float distance = 1000;
    if (initRgbUseClouds())
    {
      distance = calcMeanDistance(rgb_use_cloud_, lidar_use_cloud_);
      // cout << "mean distance:" << distance << endl;
      convertToRgbCloud(rgb_use_cloud_, rgb_use_cloud_xyzrgb_, Eigen::Vector3i(255, 0, 0));
      convertToRgbCloud(lidar_use_cloud_, lidar_use_cloud_xyzrgb_, Eigen::Vector3i(0, 0, 255));
      // showClouds(rgb_use_cloud_xyzrgb_, lidar_use_cloud_xyzrgb_);
    }
    return distance;
  }

  /*************************************************color2d*********************************************************************/
  void Calibrating::color2d(const bool use_depth, const string info)
  {
    Mat im_color;
    Mat imdepth8_true;
    imdepth_true_.convertTo(imdepth8_true, CV_8UC1, 1.0 / 256);
    imdepth8_true = imdepth8_true(rect_);
    if (use_depth)
    {
      applyColorMap(imdepth8_true, im_color, COLORMAP_JET);
    }
    else
    {
      applyColorMap(imIntensity8_, im_color, COLORMAP_JET); //!
    }
    Mat im_show = 0.8 * imRGB_ + 0.8 * im_color;
    //设置绘制文本的相关参数
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 1;
    int thickness = 2;
    int baseline;
    //获取文本框的长宽
    cv::Size text_size = cv::getTextSize(info, font_face, font_scale, thickness, &baseline);
    //将文本框居中绘制
    cv::Point origin;
    origin.x = text_size.width / 2;
    origin.y = 15 + text_size.height / 2;
    cv::putText(im_show, info, origin, font_face, font_scale, cv::Scalar(0, 0, 0), thickness, 8, 0);
    std::string img_name = info + ".jpg";
    cv::imwrite(img_name, im_show);
#ifdef debug
    cv::imshow("projection", im_show);
    cv::waitKey();
#endif
  }

  /******************************************************color3d****************************************************************/
  void Calibrating::color3d(const Vector6d &calib_params, const int density, const string pcd_path)
  {
    cout << "Calibration color3d" << endl;
    std::vector<cv::Point3f> pts_3d;
    Eigen::AngleAxisd rotation_vector3;
    rotation_vector3 = Eigen::AngleAxisd(calib_params[0], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(calib_params[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(calib_params[2], Eigen::Vector3d::UnitX());
    for (size_t i = 0; i < horizon_cloud_->size(); ++i)
    {
      pcl::PointXYZI point_3d = horizon_cloud_->points[i];
      pts_3d.emplace_back(cv::Point3f(point_3d.x, point_3d.y, point_3d.z));
    }
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
    cv::Mat distortion_coeff = (cv::Mat_<double>(1, 5) << k1_, k2_, p1_, p2_, k3_);
    cv::Mat r_vec = (cv::Mat_<double>(3, 1) << rotation_vector3.angle() * rotation_vector3.axis().transpose()[0], rotation_vector3.angle() * rotation_vector3.axis().transpose()[1], rotation_vector3.angle() * rotation_vector3.axis().transpose()[2]);
    cv::Mat t_vec = (cv::Mat_<double>(3, 1) << calib_params[3], calib_params[4], calib_params[5]);
    // project 3d-points into image view
    std::vector<cv::Point2f> pts_2d;
    cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff, pts_2d);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    int min_x = 10;
    int max_x = origin_RGB_.cols - 10;
    int min_y = 10;
    int max_y = origin_RGB_.rows - 10;
    for (size_t i = 0; i < pts_2d.size(); i = i + density)
    {
      if (pts_2d[i].x > min_x && pts_2d[i].x < max_x && pts_2d[i].y > min_y && pts_2d[i].y < max_y)
      {
        Scalar color = origin_RGB_.at<Vec3b>(pts_2d[i]);
        if (color[0] == 0 && color[1] == 0 && color[2] == 0)
        {
          continue;
        }
        if (pts_3d[i].x > 100)
        {
          continue;
        }
        pcl::PointXYZRGB p;
        p.x = pts_3d[i].x;
        p.y = pts_3d[i].y;
        p.z = pts_3d[i].z;
        p.b = origin_RGB_.at<Vec3b>(pts_2d[i].y, pts_2d[i].x)[0];
        p.g = origin_RGB_.at<Vec3b>(pts_2d[i].y, pts_2d[i].x)[1];
        p.r = origin_RGB_.at<Vec3b>(pts_2d[i].y, pts_2d[i].x)[2];
        cloud->points.push_back(p);
      }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    pcl::io::savePCDFile(pcd_path, *cloud);
  }

  /********************************************************backprojection**************************************************************/
  Point3f Calibrating::backprojection(Point2f p, float z)
  {
    int u = p.x + 90; // change 90
    int v = p.y + 90;
    Point3f p1;
    p1.x = (u - cx_) / fx_ * z;
    p1.y = (v - cy_) / fy_ * z;
    p1.z = z;
    return p1;
  }

  /******************************************************unproject_all****************************************************************/
  //全图反投影为3D点的集合
  vector<Point3f> Calibrating::unproject_all(Mat &imdepth_)
  {
    int rows = imdepth_.rows;
    int cols = imdepth_.cols * imdepth_.channels();
    vector<Point3f> pp;
    for (int v = 0; v < rows; v++)
    {
      float *data = imdepth_.ptr<float>(v);
      for (int u = 0; u < cols; u++)
      {
        float z = data[u];
        if (z <= 3 || z > 130) //只考虑深度有效的点
          continue;
        Point3f p0;
        p0 = backprojection(Point2f(u, v), z);
        pp.push_back(p0);
      }
    }
    return pp;
  }

  /***************************************************showClouds*******************************************************************/
  // 显示原始2D点云
  void Calibrating::showClouds(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud1, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud2)
  {
    // 存储useful points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // 原始点云数据显示
    show_cloud->width = cloud1->points.size() + cloud2->points.size();
    show_cloud->height = 1;
    for (size_t i = 0; i < cloud1->points.size(); i++)
    {
      show_cloud->points.push_back(cloud1->points.at(i));
    }
    for (size_t i = 0; i < cloud2->points.size(); i++)
    {
      show_cloud->points.push_back(cloud2->points.at(i));
    }

    show_cloud->width = show_cloud->points.size();
    show_cloud->height = 1;
    std::cout << "show camera(red) and horizon(blue) point clouds" << std::endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Org Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(show_cloud);
    viewer->addPointCloud(show_cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); // 设置点云大小

    while (!viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }

  /************************************************showOrgPointClouds**********************************************************************/
  void Calibrating::showOrgPointClouds(const string name)
  {
    // 存储useful points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // 原始点云数据显示
    show_cloud->width = lidar_cloud_xyzrgb_->points.size() + rgb_cloud_xyzrgb_->points.size();
    show_cloud->height = 1;
    for (size_t i = 0; i < rgb_cloud_xyzrgb_->points.size(); i++)
    {
      show_cloud->points.push_back(rgb_cloud_xyzrgb_->points.at(i));
    }
    for (size_t i = 0; i < lidar_cloud_xyzrgb_->points.size(); i++)
    {
      show_cloud->points.push_back(lidar_cloud_xyzrgb_->points.at(i));
    }
    show_cloud->width = show_cloud->points.size();
    show_cloud->height = 1;
    std::cout << "show org point clouds" << std::endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(name));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(show_cloud);
    viewer->addPointCloud(show_cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); // 设置点云大小

    while (!viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }

  /*******************************************************showNearestPointClouds***************************************************************/
  void Calibrating::showNearestPointClouds(float radius)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr kd_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < rgb_use_cloud_->size(); i++)
    {
      pcl::PointXYZRGB point;
      point.x = rgb_use_cloud_->points[i].x;
      point.y = rgb_use_cloud_->points[i].y;
      point.z = rgb_use_cloud_->points[i].z;
      point.r = 255;
      point.g = 0;
      point.b = 0;
      point.a = 255;
      show_cloud->push_back(point);
    }
    for (size_t i = 0; i < lidar_use_cloud_->size(); i++)
    {
      pcl::PointXYZRGB point;
      point.x = lidar_use_cloud_->points[i].x;
      point.y = lidar_use_cloud_->points[i].y;
      point.z = lidar_use_cloud_->points[i].z;
      point.r = 0;
      point.g = 255;
      point.b = 0;
      point.a = 255;
      show_cloud->push_back(point);
    }
    cout << "depth size:" << lidar_use_cloud_->points.size() << std::endl;
    // just for test
    // float sum_distance1 = calcMeanDistance(rgb_use_cloud_, lidar_use_cloud_);
    for (int i = 0; i < contours_imdepth_.size(); i++)
    {
      for (int j = 0; j < contours_imdepth_[i].size(); j++)
      {
        pcl::PointXYZRGB point;
        pcl::PointXYZ p;
        point.x = contours_imdepth_[i][j].x;
        point.y = contours_imdepth_[i][j].y;
        point.z = 0;
        point.a = 255;
        point.r = 0;
        point.g = 0;
        point.b = 255;
        show_cloud->points.push_back(point);
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        cloud3->points.push_back(p);
      }
    }
    cloud3->width = cloud3->points.size();
    cloud3->height = 1;
    float sum_distance2 = calcMeanDistance(rgb_use_cloud_, cloud3);
    // cout << "current distance: " << sum_distance1 << std::endl;

    cout << "org distance: " << sum_distance2 << std::endl;
    show_cloud->width = show_cloud->points.size();
    show_cloud->height = 1;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Knearest Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(show_cloud);
    viewer->addPointCloud(show_cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); // 设置点云大小

    while (!viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }

  /***********************************************calcDistance***********************************************************************/
  float Calibrating::calcDistance(pcl::PointXYZ p1, pcl::PointXYZ p2)
  {
    float distance = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    return distance;
  }

  /****************************************************calcMeanDistance******************************************************************/
  float Calibrating::calcMeanDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2)
  {
    // test
    // ofstream plot("../rgb.txt");
    // ofstream plot2("../lidar.txt");
    float mean_distance = 0;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud2);
    // 指定近邻个数
    int K = 2;
    // 创建两个向量，分别存放近邻的索引值、近邻的中心距
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    int count = 0;
    pcl::PointXYZ searchPoint;
    // cout << "(rgb size,depth size):" << cloud1->size() << " " << cloud2->size()
    //      << endl;
    Mat connect_img = Mat::zeros(imRGB_.size(), CV_8UC3);
    int line_count = 0;
    for (size_t i = 0; i < cloud1->points.size(); i++)
    {
      searchPoint = cloud1->points[i];
      // cout << "y,x:"<<searchPoint.y<<","<<searchPoint.x<<endl;
      if (tree->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        for (int j = 0; j < K; j++)
        {
          float distance =
              calcDistance(searchPoint, cloud2->points[pointIdxNKNSearch[j]]);
          int k = 1;
          if (distance < 20)
          {
            mean_distance += k * distance;
            count = count + k;
          }
        }
      }
    }

#ifdef debug
// imshow("connect", connect_img);
// cv::waitKey();
#endif
    float un_match = (2 * rgb_cloud_xyz_->size() - count) * 1.0 / (2 * rgb_cloud_xyz_->size());
    cout << "ave distance:" << mean_distance / count << " un_match:" << un_match * 5 << " cost: " << (mean_distance / count + un_match * 5) << endl;
    return (mean_distance / count + un_match * 5);
  }

  /*********************************************************testRegister*************************************************************/
  void Calibrating::testRegister(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
  {
    cout << "Calibration testRegister" << endl;
    float sum_distance1 = calcMeanDistance(cloud1, cloud2);
    for (size_t i = 0; i < cloud1->points.size(); i++)
    {
      cloud1->points[i].y = cloud1->points[i].y - 5;
    }
    float sum_distance2 = calcMeanDistance(cloud1, cloud2);
    cout << "sum distance1: " << sum_distance1 << std::endl;
    cout << "sum distance2: " << sum_distance2 << std::endl;
  }

  /**************************************************initRgbUseClouds********************************************************************/
  bool Calibrating::initRgbUseClouds()
  {
    rgb_use_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    lidar_use_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(rgb_cloud_xyz_);
    // 存储useful points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // 以depth点云作为查找中心，领域法保留点云
    pcl::PointXYZ searchPoint;
    vector<uint16_t> depth_cloud_index;
    vector<uint16_t> rgb_cloud_index;
    depth_cloud_index.resize(lidar_cloud_xyz_->size());
    for (size_t i = 0; i < depth_cloud_index.size(); i++)
    {
      depth_cloud_index[i] = 0;
    }
    rgb_cloud_index.resize(rgb_cloud_xyz_->size());
    for (size_t i = 0; i < rgb_cloud_index.size(); i++)
    {
      rgb_cloud_index[i] = 0;
    }
    for (size_t i = 0; i < lidar_cloud_xyz_->size(); i++)
    {
      searchPoint.x = lidar_cloud_xyz_->points[i].x;
      searchPoint.y = lidar_cloud_xyz_->points[i].y;
      searchPoint.z = lidar_cloud_xyz_->points[i].z;
      // 创建两个向量，分别存放近邻的索引值、近邻的中心距
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      // 指定近邻个数
      int K = 2;
      int distacne_threshold = 30;
      // 创建两个向量，分别存放近邻的索引值、近邻的中心距
      std::vector<int> pointIdxNKNSearch(K);
      std::vector<float> pointNKNSquaredDistance(K);

      if (tree->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        bool isAdd = false;
        for (int j = 0; j < K; j++)
        {
          float distance = calcDistance(searchPoint, rgb_cloud_xyz_->points[pointIdxNKNSearch[j]]);
          if (distance < distacne_threshold)
          {
            rgb_cloud_index[pointIdxNKNSearch[j]]++;
            depth_cloud_index[i]++;
          }
        }
      }
    }
    for (size_t i = 0; i < depth_cloud_index.size(); i++)
    {
      if (depth_cloud_index[i] != 0)
      {
        lidar_use_cloud_->points.push_back(lidar_cloud_xyz_->points[i]);
      }
    }
    for (size_t i = 0; i < rgb_cloud_index.size(); i++)
    {
      if (rgb_cloud_index[i] != 0)
      {
        rgb_use_cloud_->points.push_back(rgb_cloud_xyz_->points[i]);
      }
    }
    // register point clouds
    rgb_use_cloud_->width = cloud1->points.size();
    rgb_use_cloud_->height = 1;
    lidar_use_cloud_->width = cloud2->points.size();
    lidar_use_cloud_->height = 1;
    if (lidar_use_cloud_->points.size() == 0 || rgb_use_cloud_->points.size() == 0)
    {
      return false;
    }
    return true;
  }

  //**************************************************************************************************************************************************/

  void Calibrating::SetLoopCloser(LoopClosing *pLoopCloser) //system中调用
  {
    mpLoopCloser = pLoopCloser;
  }

  void Calibrating::SetTracker(Tracking *pTracker)
  {
    mpTracker = pTracker;
  }

  void Calibrating::SetLocalMapper(LocalMapping *pLocalMapper)
  {
    mpLocalMapper = pLocalMapper;
  }

  // update parameter 拋棄異常值（超過上線，或者超過下線），根據均值可信範圍，選擇是否接受標定的結果並跟新進配置文件
  //!  ---------------------------------------------------  Run  -------------------------------------------------------------------------- //
  void Calibrating::Run()
  {
    mbFinished = false;
    /******************************/
    float min_distance = 10000;
    int search_round = 3;
    int single_search_count = 20;
    Eigen::Vector3d optimize_round(0, 1, 2);

    while (1)
    {
      if (mbCalibrating)
      {
        //generate 10s' pcd file and a RGB image for calibration
        //Initialize the Calibrating thread and launch
        // Calibrating *mpCalibratingter;
        // std::thread *mptCalibrating;
        // mpCalibratingter = new Calibrating(strSettingPath,RGBPath,PcdPath,projectionType,isEnhanceImg,isFillImg);
        // mptCalibrating = new thread(&Camvox::Calibrating::Run, mpCalibratingter);
        cout << "Calibrating processing" << endl;
        mbCalibrating = false;
        
      }
      if (cumulative_flag) //TODO
      {
        // optimizing extrinsic parameters
        //mbOptimizing = false;
        cumulative_flag = false;
        initialize(RGBPath,PcdPath,projectionType,isEnhanceImg,isFillImg);
        cout << "start iteration!" << endl;
        int iter_num = 15;
        float single_resolution = deg2rad(0.05);
        best_r_ = best_r_ + deg2rad(-0.4);
        best_p_ = best_p_ + deg2rad(-0.4);
        best_y_ = best_y_ + deg2rad(0.15);
        optimize_type = 0;
        double test_angle[1] = {best_r_};
        float init_distance = costFunction(test_angle);
        std::string init_info = "./camvox/debug/cost:" + std::to_string(init_distance);
        color2d(color2dDepth, init_info);
        for (int iter_count = 0; iter_count < search_round; iter_count++)
        {
          for (int round_index = 0; round_index < 3; round_index++)
          {
            optimize_type = optimize_round[round_index];
            if (optimize_type == 0)
            {
              float min_distance = 1000;
              double angle[1] = {best_r_};
              double best_roll = angle[0];
              for (int i = 0; i < single_search_count; i++)
              {
                float roll = angle[0] + pow(-1, i) * int(i / 2) * single_resolution;
                double test_angle[1] = {roll};
                float distance = costFunction(test_angle);
                if (min_distance > distance)
                {
                  min_distance = distance;
                  best_roll = roll;
                }
              }
              best_r_ = best_roll;
              double test_angle[1] = {best_r_};
              float distance = costFunction(test_angle);
              std::string info = "./camvox/debug/cost:" + std::to_string(distance);
              color2d(color2dDepth, info);
            }
            else if (optimize_type == 1)
            {
              float min_distance = 1000;
              double angle[1] = {best_p_};
              double best_pitch = angle[0];
              for (int i = 0; i < single_search_count; i++)
              {
                float pitch = angle[0] + pow(-1, i) * int(i / 2) * single_resolution;
                double test_angle[1] = {pitch};
                float distance = costFunction(test_angle);
                if (min_distance > distance)
                {
                  min_distance = distance;
                  best_pitch = pitch;
                }
              }
              best_p_ = best_pitch;
              double test_angle[1] = {best_p_};
              float distance = costFunction(test_angle);
              std::string info = "./camvox/debug/cost:" + std::to_string(distance);
              color2d(color2dDepth, info);
            }
            else if (optimize_type == 2)
            {
              float min_distance = 1000;
              double angle[1] = {best_y_};
              double best_yaw = angle[0];
              for (int i = 0; i < single_search_count; i++)
              {
                float yaw = angle[0] + pow(-1, i) * int(i / 2) * single_resolution;
                double test_angle[1] = {yaw};
                float distance = costFunction(test_angle);
                if (min_distance > distance)
                {
                  min_distance = distance;
                  best_yaw = yaw;
                }
              }
              best_y_ = best_yaw;
              double test_angle[1] = {best_y_};
              float distance = costFunction(test_angle);
              std::string info = "./camvox/debug/cost:" + std::to_string(distance);
              color2d(color2dDepth, info);
            }
          }
          cout << "After round" << iter_count << " best rpy:" << rad2deg(best_r_) << "," << rad2deg(best_p_) << "," << rad2deg(best_y_) << endl;
          optimize_type = 0;
          double test_angle[1] = {best_r_};
          cout << "final distance:" << costFunction(test_angle) << endl;
          std::string round_numstr = "round " + std::to_string(iter_count);
          //showOrgPointClouds("optimize rpy");
          //showClouds(rgb_use_cloud_xyzrgb_,lidar_use_cloud_xyzrgb_);
          single_resolution = single_resolution / 2;
        }
      }
      ResetIfRequested();
      if (CheckFinish())
        break;
      usleep(3000);
    }
    SetFinish();
  } // namespace Camvox
  //!  --------------------------------------------------------------------------------------------------------------------------------------- //
  //! Thread Synch
  void Calibrating::RequestStop()
  {
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    //  unique_lock<mutex> lock2(mMutexNewKFs);
    //  mbAbortBA = true;
  }

  bool Calibrating::Stop()
  {
    unique_lock<mutex> lock(mMutexStop);
    if (mbStopRequested && !mbNotStop)
    {
      mbStopped = true;
      cout << "Calibrating STOP" << endl;
      return true;
    }
    return false;
  }

  bool Calibrating::isStopped()
  {
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
  }

  bool Calibrating::stopRequested()
  {
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
  }

  void Calibrating::Release()
  {
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if (mbFinished)
      return;
    mbStopped = false;
    mbStopRequested = false;
    //for (list<KeyFrame *>::iterator lit = mlNewKeyFrames.begin(), lend = mlNewKeyFrames.end(); lit != lend; lit++)
    //    delete *lit;
    //mlNewKeyFrames.clear();

    cout << "Calibrating RELEASE" << endl;
  }

  bool Calibrating::SetNotStop(bool flag)
  {
    unique_lock<mutex> lock(mMutexStop);

    if (flag && mbStopped)
      return false;

    mbNotStop = flag;

    return true;
  }

  //! Reset
  void Calibrating::RequestReset()
  {
    {
      unique_lock<mutex> lock(mMutexReset);
      mbResetRequested = true;
    }
    while (1)
    {
      {
        unique_lock<mutex> lock2(mMutexReset);
        if (!mbResetRequested)
          break;
      }
      usleep(5000);
    }
  }

  void Calibrating::ResetIfRequested()
  {
    unique_lock<mutex> lock(mMutexReset);
    if (mbResetRequested)
    {
      //mlpLoopKeyFrameQueue.clear();
      //mLastLoopKFid = 0;
      mbResetRequested = false;
    }
  }

  //! mutex
  void Calibrating::RequestFinish()
  {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
  }

  bool Calibrating::CheckFinish()
  {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
  }

  void Calibrating::SetFinish()
  {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
  }

  bool Calibrating::isFinished()
  {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
  }

  void Calibrating::InformCalibrating(const bool &flag)
  {
    mbCalibrating = flag;
    mbOptimizing = flag;
  }

} // namespace Camvox