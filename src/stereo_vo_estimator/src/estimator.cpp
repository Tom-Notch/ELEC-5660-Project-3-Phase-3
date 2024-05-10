#include "estimator.h"

void drawImage(const cv::Mat& img, const vector<cv::Point2f>& pts, string name)
{
  auto draw = img.clone();
  for (unsigned int i = 0; i < pts.size(); i++)
  {
    cv::circle(draw, pts[i], 2, cv::Scalar(0, 255, 0), -1, 8);
  }
  cv::imshow(name, draw);
  cv::waitKey(1);
}

Estimator::Estimator()
{
  ROS_INFO("Estimator init begins.");
  prev_frame.frame_time = ros::Time(0.0);
  prev_frame.w_t_c = Eigen::Vector3d(0, 0, 0);
  prev_frame.w_R_c = Eigen::Matrix3d::Identity();
  fail_cnt = 0;
  init_finish = false;
}

void Estimator::reset()
{
  ROS_ERROR("Lost, reset!");
  key_frame = prev_frame;
  fail_cnt = 0;
  init_finish = false;
}

void Estimator::setParameter()
{
  for (int i = 0; i < 2; i++)
  {
    tic[i] = TIC[i];
    ric[i] = RIC[i];
    cout << " extrinsic cam " << i << endl
         << ric[i] << endl
         << tic[i].transpose() << endl;
  }

  prev_frame.frame_time = ros::Time(0.0);
  prev_frame.w_t_c = tic[0];
  prev_frame.w_R_c = ric[0];
  key_frame = prev_frame;

  readIntrinsicParameter(CAM_NAMES);

  // transform between left and right camera
  Matrix4d Tl, Tr;
  Tl.setIdentity();
  Tl.block(0, 0, 3, 3) = ric[0];
  Tl.block(0, 3, 3, 1) = tic[0];
  Tr.setIdentity();
  Tr.block(0, 0, 3, 3) = ric[1];
  Tr.block(0, 3, 3, 1) = tic[1];
  Tlr = Tl.inverse() * Tr;
}

void Estimator::readIntrinsicParameter(const vector<string>& calib_file)
{
  // ! I have to do this because camodocal::Camera does not have access function for intrinsics matrix...
  this->m_left_camera_intrinsic_matrix = cv::Mat_<double>(3, 3);
  cv::FileStorage fs(calib_file[0], cv::FileStorage::READ);

  const double& fx = fs["projection_parameters"]["fx"];
  const double& fy = fs["projection_parameters"]["fy"];
  const double& cx = fs["projection_parameters"]["cx"];
  const double& cy = fs["projection_parameters"]["cy"];

  this->m_left_camera_intrinsic_matrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

  for (size_t i = 0; i < calib_file.size(); i++)
  {
    ROS_INFO("reading parameter of camera %s", calib_file[i].c_str());
    camodocal::CameraPtr camera =
        camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
    m_camera.push_back(camera);
  }
}

bool Estimator::inputImage(ros::Time time_stamp, const cv::Mat& _img, const cv::Mat& _img1)
{
  if (fail_cnt > 20)
  {
    reset();
  }
  std::cout << "receive new image===========================" << std::endl;

  Estimator::frame cur_frame;
  cur_frame.frame_time = time_stamp;
  cur_frame.img = _img;

  // cv::imshow("img", _img);
  // cv::waitKey(1);

  vector<cv::Point2f> left_pts_2d, right_pts_2d;
  vector<cv::Point3f> key_pts_3d;

  c_R_k.setIdentity();
  c_t_k.setZero();

  if (init_finish)
  {
    // To do: match features between the key frame and the current left image
    this->trackFeatureBetweenFrames(key_frame, cur_frame.img, key_pts_3d, left_pts_2d);

    // To do: undistort the points of the left image and compute relative motion with the key frame.
    left_pts_2d = this->undistortedPts(left_pts_2d, m_camera[0]);
    this->estimateTBetweenFrames(key_pts_3d, left_pts_2d, c_R_k, c_t_k);
    left_pts_2d.clear();
  }

  // To do: extract new features for the current frame.
  this->extractNewFeatures(_img, cur_frame.uv);

  // To do: compute the camera pose of the current frame.
  if (init_finish)
  {
    const Matrix4d& w_T_key_frame_c = SE3_from_R_t(key_frame.w_R_c, key_frame.w_t_c);
    const Matrix4d& k_T_cur_frame_c = SE3_from_R_t(c_R_k, c_t_k);
    const Matrix4d& w_T_cur_frame_c = w_T_key_frame_c * k_T_cur_frame_c;
    cur_frame.w_R_c = w_T_cur_frame_c.block<3, 3>(0, 0);
    cur_frame.w_t_c = w_T_cur_frame_c.block<3, 1>(0, 3);

    // ROS_INFO_STREAM("cur_frame.w_R_c: " << endl
    //                                     << cur_frame.w_R_c);
    // ROS_INFO_STREAM("cur_frame.w_t_c: " << endl
    //                                     << cur_frame.w_t_c);
  }
  else
  {
    cur_frame.w_R_c = c_R_k;
    cur_frame.w_t_c = c_t_k;
  }

  // ROS_INFO_STREAM("c_T_k: " << endl
  //                           << c_R_k);
  // ROS_INFO_STREAM("c_t_k: " << endl
  //                           << c_t_k);

  // To do: undistort the 2d points of the current frame and generate the corresponding 3d points.
  this->trackFeatureLeftRight(_img, _img1, cur_frame.uv, right_pts_2d);
  left_pts_2d = this->undistortedPts(cur_frame.uv, m_camera[0]);
  right_pts_2d = this->undistortedPts(right_pts_2d, m_camera[1]);
  this->generate3dPoints(left_pts_2d, right_pts_2d, cur_frame.xyz, cur_frame.uv);

  // Change key frame
  if (c_t_k.norm() > TRANSLATION_THRESHOLD || acos(Quaterniond(c_R_k).w()) * 2.0 > ROTATION_THRESHOLD || key_pts_3d.size() < FEATURE_THRESHOLD || !init_finish)
  {
    key_frame = cur_frame;
    ROS_INFO("Change key frame to current frame.");
  }

  prev_frame = cur_frame;

  updateLatestStates(cur_frame);

  init_finish = true;

  return true;
}

const Matrix4d Estimator::SE3_from_R_t(const Matrix3d R, const Vector3d t)
{
  Matrix4d T;
  T.setIdentity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  return T;
}

bool Estimator::trackFeatureBetweenFrames(const Estimator::frame& keyframe, const cv::Mat& cur_img,
                                          vector<cv::Point3f>& key_pts_3d,
                                          vector<cv::Point2f>& cur_pts_2d)
{
  // To do: track features between the key frame and the current frame to obtain corresponding 2D, 3D points.
  key_pts_3d.clear();
  cur_pts_2d.clear();

  const cv::Mat& keyframe_img = keyframe.img;

  const vector<cv::Point2f>& left_pts_2d = keyframe.uv;   // all 2D features in key frame
  const vector<cv::Point3f>& left_pts_3d = keyframe.xyz;  // all 3D feature points in key frame
  vector<cv::Point2f> right_pts;

  right_pts.clear();
  right_pts.resize(left_pts_2d.size());

  vector<uchar> status;
  vector<float> err;

  calcOpticalFlowPyrLK(keyframe_img,
                       cur_img,
                       left_pts_2d,
                       right_pts,
                       status,
                       err,
                       cv::Size(21, 21),
                       3,
                       cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                       0,
                       0.001);

  // Filter out the points with bad status
  for (size_t i = 0; i < status.size(); ++i)
  {
    if (status[i])
    {
      key_pts_3d.push_back(left_pts_3d[i]);
      cur_pts_2d.push_back(right_pts[i]);
    }
  }

  // Check if there are enough points tracked
  if (cur_pts_2d.size() < MIN_CNT)
  {
    return false;
  }

  return true;
}

bool Estimator::estimateTBetweenFrames(vector<cv::Point3f>& key_pts_3d,
                                       vector<cv::Point2f>& cur_pts_2d, Matrix3d& R, Vector3d& t)
{
  // To do: calculate relative pose between the key frame and the current frame using the matched 2d-3d points

  // Outputs from solvePnPRansac
  cv::Mat rvec, tvec;
  vector<int> inliers;

  // RANSAC parameters
  float reprojectionError = 3.0f;  // Example reprojection error threshold (in pixels)
  double confidence = 0.99;        // Confidence level, between 0 and 1
  int maxIters = 100;              // Maximum number of iterations

  // Call solvePnPRansac without distortion coefficients since keypoints are undistorted
  bool success = solvePnPRansac(key_pts_3d, cur_pts_2d, this->m_left_camera_intrinsic_matrix, cv::noArray(),
                                rvec, tvec, false, maxIters, reprojectionError, confidence, inliers, cv::SOLVEPNP_ITERATIVE);

  if (!success || inliers.empty())
  {
    return false;  // solvePnPRansac failed to find a solution or no inliers found
  }

  // Convert rotation vector to rotation matrix
  cv::Mat Rmat;
  cv::Rodrigues(rvec, Rmat);

  // Convert OpenCV Mat to Eigen Matrix
  cv::cv2eigen(Rmat, R);
  cv::cv2eigen(tvec, t);

  return true;
}

void Estimator::extractNewFeatures(const cv::Mat& img, vector<cv::Point2f>& uv)
{
  // TODO
  uv.clear();

  // Parameters for Shi-Tomasi algorithm
  double qualityLevel = 0.01;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  // Apply corner detection
  cv::goodFeaturesToTrack(img,
                          uv,
                          MAX_CNT,
                          qualityLevel,
                          MIN_DIST,
                          cv::Mat(),  // mask - you could pass a mask here if you want
                          blockSize,
                          useHarrisDetector,
                          k);
  return;
}

bool Estimator::trackFeatureLeftRight(const cv::Mat& _img, const cv::Mat& _img1,
                                      vector<cv::Point2f>& left_pts, vector<cv::Point2f>& right_pts)
{
  // TODO: track features left to right frame and obtain corresponding 2D points.
  vector<uchar> status;
  vector<float> err;

  // Check if there are enough points to track
  if (left_pts.size() < MIN_CNT)
  {
    return false;
  }

  calcOpticalFlowPyrLK(_img,
                       _img1,
                       left_pts,
                       right_pts,
                       status,
                       err,
                       cv::Size(21, 21),
                       3,
                       cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                       0,
                       0.001);

  // Filter out the points with bad status
  vector<cv::Point2f> left_pts_good;
  vector<cv::Point2f> right_pts_good;
  for (size_t i = 0; i < status.size(); ++i)
  {
    if (status[i])
    {
      left_pts_good.push_back(left_pts[i]);
      right_pts_good.push_back(right_pts[i]);
    }
  }

  // Check if there are enough points tracked
  if (left_pts_good.size() < MIN_CNT)
  {
    return false;
  }

  left_pts.clear();
  left_pts = left_pts_good;
  right_pts.clear();
  right_pts = right_pts_good;

  if (SHOW_FEATURE)
  {
    this->left_right_tracking_vis = visualizeTracking(_img, _img1, left_pts_good, right_pts_good);
  }

  return true;
}

const cv::Mat Estimator::visualizeTracking(const cv::Mat& left_img, const cv::Mat& right_img, const vector<cv::Point2f>& left_pts, const vector<cv::Point2f>& right_pts) const
{
  // Convert grayscale images to RGB
  cv::Mat left_img_color, right_img_color;
  cvtColor(left_img, left_img_color, cv::COLOR_GRAY2BGR);
  cvtColor(right_img, right_img_color, cv::COLOR_GRAY2BGR);

  // Create a large image to hold both the left and right images side by side
  cv::Mat composite_img(left_img_color.rows, left_img_color.cols + right_img_color.cols, left_img_color.type());

  // Copy left image to the left side of the composite image
  left_img_color.copyTo(composite_img(cv::Rect(0, 0, left_img_color.cols, left_img_color.rows)));

  // Copy right image to the right side of the composite image
  right_img_color.copyTo(composite_img(cv::Rect(left_img_color.cols, 0, right_img_color.cols, right_img_color.rows)));

  // Draw lines between corresponding points
  for (size_t i = 0; i < left_pts.size(); i++)
  {
    // Offset the right points by the width of the left image
    cv::Point2f right_pt_offset = right_pts[i] + cv::Point2f(static_cast<float>(left_img_color.cols), 0.0f);

    // Random color for each line
    cv::Scalar color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);

    // Draw line and circles for each correspondence
    line(composite_img, left_pts[i], right_pt_offset, color);
    // circle(composite_img, left_pts[i], 5, color, -1);
    // circle(composite_img, right_pt_offset, 5, color, -1);
  }

  return composite_img;
}

void Estimator::generate3dPoints(const vector<cv::Point2f>& left_pts,
                                 const vector<cv::Point2f>& right_pts,
                                 vector<cv::Point3f>& cur_pts_3d,
                                 vector<cv::Point2f>& cur_pts_2d)
{
  Eigen::Matrix<double, 3, 4> P1, P2;

  P1 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0;
  P2.block(0, 0, 3, 3) = (Tlr.block(0, 0, 3, 3).transpose());
  P2.block(0, 3, 3, 1) = -P2.block(0, 0, 3, 3) * Tlr.block(0, 3, 3, 1);

  vector<uchar> status;

  for (unsigned int i = 0; i < left_pts.size(); ++i)
  {
    Vector2d pl(left_pts[i].x, left_pts[i].y);
    Vector2d pr(right_pts[i].x, right_pts[i].y);
    Vector3d pt3;
    triangulatePoint(P1, P2, pl, pr, pt3);

    if (pt3[2] > 0)
    {
      cur_pts_3d.push_back(cv::Point3f(pt3[0], pt3[1], pt3[2]));
      status.push_back(1);
    }
    else
    {
      status.push_back(0);
    }
  }

  reduceVector<cv::Point2f>(cur_pts_2d, status);
}

bool Estimator::inBorder(const cv::Point2f& pt, const int& row, const int& col)
{
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y &&
         img_y < row - BORDER_SIZE;
}

double Estimator::distance(cv::Point2f pt1, cv::Point2f pt2)
{
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}

template <typename Derived>
void Estimator::reduceVector(vector<Derived>& v, vector<uchar> status)
{
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}

void Estimator::updateLatestStates(frame& latest_frame)
{
  // To do: update the latest_time, latest_pointcloud, latest_P, latest_Q, latest_rel_P and latest_rel_Q.
  // latest_P and latest_Q should be the pose of the body (IMU) in the world frame.
  // latest_rel_P and latest_rel_Q should be the relative pose of the current body frame relative to the body frame of the key frame.
  // latest_pointcloud should be in the current camera frame.

  latest_time = latest_frame.frame_time;

  latest_P = latest_frame.w_t_c;
  latest_Q = Quaterniond(latest_frame.w_R_c);

  latest_rel_P = c_t_k;
  latest_rel_Q = Quaterniond(c_R_k);

  latest_pointcloud = latest_frame.xyz;
}

void Estimator::triangulatePoint(Eigen::Matrix<double, 3, 4>& Pose0, Eigen::Matrix<double, 3, 4>& Pose1,
                                 Eigen::Vector2d& point0, Eigen::Vector2d& point1,
                                 Eigen::Vector3d& point_3d)
{
  Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
  design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
  design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
  design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
  design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
  Eigen::Vector4d triangulated_point;
  triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
  point_3d(0) = triangulated_point(0) / triangulated_point(3);
  point_3d(1) = triangulated_point(1) / triangulated_point(3);
  point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

double Estimator::reprojectionError(Matrix3d& R, Vector3d& t, cv::Point3f& key_pts_3d, cv::Point2f& cur_pts_2d)
{
  Vector3d pt1(key_pts_3d.x, key_pts_3d.y, key_pts_3d.z);
  Vector3d pt2 = R * pt1 + t;
  pt2 = pt2 / pt2[2];
  return sqrt(pow(pt2[0] - cur_pts_2d.x, 2) + pow(pt2[1] - cur_pts_2d.y, 2));
}

vector<cv::Point2f> Estimator::undistortedPts(vector<cv::Point2f>& pts, camodocal::CameraPtr cam)
{
  vector<cv::Point2f> un_pts;
  for (unsigned int i = 0; i < pts.size(); i++)
  {
    Eigen::Vector2d a(pts[i].x, pts[i].y);
    Eigen::Vector3d b;
    cam->liftProjective(a, b);
    un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
  }
  return un_pts;
}
