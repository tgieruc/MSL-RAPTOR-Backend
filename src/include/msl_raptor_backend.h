/*
Copyright 2020 Benjamin Ramtoula
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#pragma once

#include "UKF.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <numeric>
#include <opencv2/core/eigen.hpp>
#include <vector>
#include <iostream>
#include "omp.h"
#include <tuple>
#include <math.h>

// #include <functional>

#define COS_45 0.52532198881
// -------------------------------------------------------------------
// ----------------------- START PARAMETER STRUCTS -------------------
// -------------------------------------------------------------------
namespace msl_raptor_backend
{
  typedef Eigen::Matrix<double, 7, 1> PoseVec;
  typedef Eigen::Matrix<double, 6, 1> GradVec;

  struct CameraParams
  {
    cv::Mat dist_coeffs;
    cv::Mat K;
    Eigen::Matrix3d K_inv;
    cv::Mat rvec;
    cv::Mat tvec;
    Eigen::Affine3d tf_ego_cam;
    Eigen::Affine3f tf_cam_ego;

    void init(const float ppx,
              const float ppy,
              const float fx,
              const float fy,
              const Eigen::Affine3f &extrinsics_in,
              const cv::Mat &dist_coeffs_in)
    {
      cv::Mat cam_matrix = cv::Mat::eye(3, 3, CV_32F);
      cam_matrix.at<float>(0, 0) = fx;
      cam_matrix.at<float>(1, 1) = fy;
      cam_matrix.at<float>(0, 2) = ppx;
      cam_matrix.at<float>(1, 2) = ppy;
      init(cam_matrix, extrinsics_in, dist_coeffs_in);
    }

    void init(const std::vector<float> &camera_matrix_in,
              const std::vector<float> &rvec_in,
              const std::vector<float> &tvec_in,
              const std::vector<float> &dist_coeffs_in)
    {
      cv::Mat camera_matrix = cv::Mat(camera_matrix_in, true).reshape(1, 3);
      cv::Mat rvec = cv::Mat(rvec_in, true);
      cv::Mat tvec = cv::Mat(tvec_in, true);
      cv::Mat dist_coeffs = cv::Mat(dist_coeffs_in, true);
      init(camera_matrix, rvec, tvec, dist_coeffs);
    }

    void init(const cv::Mat &camera_matrix_in,
              const  Eigen::Affine3f &extrinsics,
              const cv::Mat &dist_coeffs_in)
    {
      cv::Mat rvec(1, 3, CV_32F);
      cv::Mat tvec(1, 3, CV_32F);
      cv::Mat rmat(3, 3, CV_32F);

      Eigen::MatrixXf rmat_eigen = extrinsics.matrix().topLeftCorner<3, 3>();
      Eigen::MatrixXf tvec_eigen = extrinsics.matrix().topRightCorner<3, 1>();

      cv::eigen2cv(rmat_eigen, rmat);
      cv::eigen2cv(tvec_eigen, tvec);

      cv::Rodrigues(rmat, rvec);

      init(camera_matrix_in, rvec, tvec, dist_coeffs_in);
    }

    void init(const cv::Mat &camera_matrix_in,
              const cv::Mat &rvec_in,
              const cv::Mat &tvec_in,
              const cv::Mat &dist_coeffs_in)
    {
      K = camera_matrix_in.clone();
      Eigen::Matrix3f K_eigen;
      cv::cv2eigen(K, K_eigen);
      K_inv = K_eigen.cast<double>().inverse();
      rvec = rvec_in.clone();
      tvec = tvec_in.clone();

      cv::Mat rot_mat;
      cv::Rodrigues(rvec, rot_mat);
      Eigen::Matrix3f rot_mat_eigen;
      cv::cv2eigen(rot_mat, rot_mat_eigen);
      // ;
      tf_ego_cam = Eigen::Translation3d(double(tvec.at<float>(0)),
                                        double(tvec.at<float>(1)),
                                        double(tvec.at<float>(2))) *
                   Eigen::AngleAxisd(rot_mat_eigen.cast<double>());
      tf_cam_ego = tf_ego_cam.inverse().cast<float>();
      dist_coeffs = dist_coeffs_in.clone();
    }

    void updateIntrinsics(const float ppx,
                          const float ppy,
                          const float fx,
                          const float fy,
                          const cv::Mat &dist_coeffs_in)
    {
      cv::Mat cam_matrix = cv::Mat::eye(3, 3, CV_32F);
      cam_matrix.at<float>(0, 0) = fx;
      cam_matrix.at<float>(1, 1) = fy;
      cam_matrix.at<float>(0, 2) = ppx;
      cam_matrix.at<float>(1, 2) = ppy;
      updateIntrinsics(cam_matrix, dist_coeffs);
    }

    void updateIntrinsics(const std::vector<float> &camera_matrix_in,
                          const std::vector<float> &dist_coeffs_in)
    {
      cv::Mat camera_matrix = cv::Mat(camera_matrix_in, true).reshape(1, 3);
      cv::Mat dist_coeffs = cv::Mat(dist_coeffs_in, true);
      updateIntrinsics(camera_matrix, dist_coeffs);
    }

    void updateIntrinsics(const cv::Mat &camera_matrix_in,
                          const cv::Mat &dist_coeffs_in)
    {
      K = camera_matrix_in.clone();
      Eigen::Matrix3f K_eigen;
      cv::cv2eigen(K, K_eigen);
      K_inv = K_eigen.cast<double>().inverse();
      dist_coeffs = dist_coeffs_in.clone();
    }

    void updateExtrinsics(const Eigen::Affine3d &extrinsics)
    {
      cv::Mat rmat(3, 3, CV_32F);
      Eigen::MatrixXf rmat_eigen =
          extrinsics.matrix().topLeftCorner<3, 3>().cast<float>();
      Eigen::MatrixXf tvec_eigen =
          extrinsics.matrix().topRightCorner<3, 1>().cast<float>();

      cv::eigen2cv(rmat_eigen, rmat);
      cv::eigen2cv(tvec_eigen, tvec);

      cv::Rodrigues(rmat, rvec);
      tf_ego_cam = extrinsics;
      tf_cam_ego = tf_ego_cam.inverse().cast<float>();
    }

    void updateExtrinsics(const cv::Mat &rvec_in, const cv::Mat &tvec_in)
    {
      cv::Mat rot_mat;
      rvec = rvec_in;
      tvec = tvec_in;
      cv::Rodrigues(rvec, rot_mat);
      Eigen::Matrix3f rot_mat_eigen;
      cv::cv2eigen(rot_mat, rot_mat_eigen);
      // ;
      tf_ego_cam = Eigen::Translation3d(double(tvec.at<float>(0)),
                                        double(tvec.at<float>(1)),
                                        double(tvec.at<float>(2))) *
                   Eigen::AngleAxisd(rot_mat_eigen.cast<double>());
      tf_cam_ego = tf_ego_cam.inverse().cast<float>();
    }

    void updateExtrinsics(const std::vector<float> &rvec_in,
                          const std::vector<float> &tvec_in)
    {
      cv::Mat rvec = cv::Mat(rvec_in, true);
      cv::Mat tvec = cv::Mat(tvec_in, true);
      updateExtrinsics(rvec, tvec);
    }

    CameraParams(){};

    CameraParams(const float ppx,
                 const float ppy,
                 const float fx,
                 const float fy,
                 const Eigen::Affine3f &extrinsics,
                 const cv::Mat &dist_coeffs_in)
    {
      init(ppx, ppy, fx, fy, extrinsics, dist_coeffs_in);
    }

    CameraParams(const cv::Mat &camera_matrix_in,
                 const Eigen::Affine3f &extrinsics,
                 const cv::Mat &dist_coeffs_in)
    {
      init(camera_matrix_in, extrinsics, dist_coeffs_in);
    }

    CameraParams(const std::vector<float> &camera_matrix_in,
                 const std::vector<float> &rvec_in,
                 const std::vector<float> &tvec_in,
                 const std::vector<float> &dist_coeffs_in)
    {
      init(camera_matrix_in, rvec_in, tvec_in, dist_coeffs_in);
    }
  };

  struct ObjPoseInitParams
  {

    // For optimising the initial bounding box. One step size and learning rate per position (3) and quaternion value (4). Variable number of initial pose depths.
    GradVec grad_comp_step_sizes;
    GradVec lr;
    // Eigen::MatrixXd init_pose_guesses;
    std::vector<PoseVec> init_pose_guesses;
    double momentum;
    int max_steps;
    int conv_steps;
    int period_lower_lr;

    ObjPoseInitParams(){};

    ObjPoseInitParams(
        const std::vector<double> &grad_comp_step_sizes_in,
        const std::vector<double> &lr_in,
        const std::vector<PoseVec> &init_pose_guesses_in,
        const double momentum_in,
        const int max_steps_in,
        const int conv_steps_in,
        const int period_lower_lr_in)
    {

      grad_comp_step_sizes = GradVec::Map(
          grad_comp_step_sizes_in.data(), grad_comp_step_sizes_in.size());

      lr = GradVec::Map(lr_in.data(), lr_in.size());
      momentum = momentum_in;
      max_steps = max_steps_in;
      conv_steps = conv_steps_in;
      period_lower_lr = period_lower_lr_in;
      init_pose_guesses = init_pose_guesses_in;
    }
  };

  // Object-specific parameters, contains UKF parameters and properties associated to one object to track:
  // Initial UKF state covariance, measurement noise, process noise, object dimensions or approximate point cloud.
  struct ObjParams
  {
    Eigen::MatrixXf model_points_ado;
    Eigen::MatrixXf model_points_ado_aug_t;

    float width;
    float height;
    float length;
    float aspect_ratio;

    Eigen::VectorXd state_cov_diags;
    Eigen::VectorXd process_noise_cov_diags;
    Eigen::VectorXd measure_noise_cov_diags;

    ObjPoseInitParams obj_pose_init_params;

    ObjParams(){};

    ObjParams(const Eigen::MatrixXf &model_points_ado_in,
              const std::vector<double> &state_cov_diags_in,
              const std::vector<double> &process_noise_cov_diags_in,
              const std::vector<double> &measure_noise_cov_diags_in,
              const ObjPoseInitParams &obj_pose_init_params_in)
    {
      init(model_points_ado_in,
           state_cov_diags_in,
           process_noise_cov_diags_in,
           measure_noise_cov_diags_in,
           obj_pose_init_params_in);
    }
    ObjParams(float width,
              float height,
              float length,
              const std::vector<double> &state_cov_diags_in,
              const std::vector<double> &process_noise_cov_diags_in,
              const std::vector<double> &measure_noise_cov_diags_in,
              const ObjPoseInitParams &obj_pose_init_params_in)
    {
      float half_height = height / 2;
      float half_width = width / 2;
      float half_length = length / 2;
      Eigen::Matrix<float, 8, 3> model_points;
      model_points << half_width, half_height, half_length, //
          half_width, half_height, -half_length,            //
          half_width, -half_height, -half_length,           //
          half_width, -half_height, half_length,            //
          -half_width, -half_height, half_length,           //
          -half_width, -half_height, -half_length,          //
          -half_width, half_height, -half_length,           //
          -half_width, half_height, half_length;            //

      init(model_points,
           state_cov_diags_in,
           process_noise_cov_diags_in,
           measure_noise_cov_diags_in,
           obj_pose_init_params_in);
    }

    void init(const Eigen::MatrixXf &model_points_ado_in,
              const std::vector<double> &state_cov_diags_in,
              const std::vector<double> &process_noise_cov_diags_in,
              const std::vector<double> &measure_noise_cov_diags_in,
              const ObjPoseInitParams &obj_pose_init_params_in)
    {
      model_points_ado = model_points_ado_in;

      width =
          model_points_ado.col(0).maxCoeff() - model_points_ado.col(0).minCoeff();
      height =
          model_points_ado.col(1).maxCoeff() - model_points_ado.col(1).minCoeff();
      length =
          model_points_ado.col(2).maxCoeff() - model_points_ado.col(2).minCoeff();

      aspect_ratio = width / height;
      model_points_ado_aug_t = model_points_ado;
      model_points_ado_aug_t.conservativeResize(
          model_points_ado_aug_t.rows(), model_points_ado_aug_t.cols() + 1);
      model_points_ado_aug_t.col(model_points_ado_aug_t.cols() - 1) =
          Eigen::VectorXf::Ones(model_points_ado_aug_t.rows());
      model_points_ado_aug_t.transposeInPlace();

      state_cov_diags = Eigen::VectorXd::Map(state_cov_diags_in.data(),
                                             state_cov_diags_in.size());
      process_noise_cov_diags = Eigen::VectorXd::Map(
          process_noise_cov_diags_in.data(), process_noise_cov_diags_in.size());
      measure_noise_cov_diags = Eigen::VectorXd::Map(
          measure_noise_cov_diags_in.data(), measure_noise_cov_diags_in.size());

      obj_pose_init_params = obj_pose_init_params_in;
    }
  };
} // namespace msl_raptor_backend
// -----------------------------------------------------------------
// ----------------------- END PARAMETER STRUCTS -------------------
// -----------------------------------------------------------------

// ------------------------------------------------------------------------------------
// ----------------------- START MSL-RAPTOR Back-end UKF definition -------------------
// ------------------------------------------------------------------------------------
// The measurement can be an axis-aligned bounding box (4 parameters) or an angled one (5 parameters with the angle). The class is templated on this factor.
template <bool aligned_bb>
class MSLRaptorUKF
{
  typedef typename msl_raptor_backend::PoseVec PoseVec;
  typedef typename msl_raptor_backend::GradVec GradVec;

public:
  /** define state vector: <scalars, 3-vectors, quaternions> */
  typedef kalman::Vector<0, 3, 1> StateVec; // the layout is: (pos x 3, vel x 3,
                                            // angularvel x 3, attitude x 4)
  typedef Eigen::Affine3d InputType;
  /** define measurement vector <scalars, 3-vectors, quaternions> */
  typedef typename std::conditional<
      /*    */ aligned_bb,
      /* y? */ kalman::Vector<4, 0, 0>,
      /* n? */ kalman::Vector<5, 0, 0>>::type MeasureVec;

  static void init(kalman::UKF<MSLRaptorUKF, InputType> &ukf,
                   MSLRaptorUKF msl_raptor_ukf)
  {
    ukf.stateRootCov = msl_raptor_ukf.obj_params_.state_cov_diags.asDiagonal();
    ukf.stateRootCov =
        ukf.stateRootCov.llt()
            .matrixU(); // note that we are using sqrt of actual cov matrix

    ukf.measureNoiseRootCov =
        msl_raptor_ukf.obj_params_.measure_noise_cov_diags.asDiagonal();
    ukf.measureNoiseRootCov = ukf.measureNoiseRootCov.llt().matrixU();

    ukf.processNoiseRootCov =
        msl_raptor_ukf.obj_params_.process_noise_cov_diags.asDiagonal();
    ukf.processNoiseRootCov =
        ukf.processNoiseRootCov.llt()
            .matrixU(); // note that we are using sqrt of actual cov matrix

    ukf.ukfModel = msl_raptor_ukf;
  }

  static StateVec dF(const StateVec &state, const InputType &u)
  {
    StateVec out = state;

    /* differentiate the quaternions automatically.
     * Second argument specifies start of angular velocity params in state
     * vector */
    kalman::util::diffQuaternion(out, 6);

    /* differentiate position automatically.
     * arguments are: (output state vector, start of position param, end of
     * position param, beginning of velocity param)  */
    kalman::util::diffPosition(out, 0, 3, 3);
    return out;
  }

  // Used if bounding box is angled
  // Approximates a full UKF state (6DoF pose) from a 2D bounding box measurement. Relies on heuristics and assumptions.
  template <bool is_aligned_bb = aligned_bb>
  StateVec approxStateFromBbHeuristic(
      typename std::enable_if<!is_aligned_bb, const MeasureVec &>::type bb)
  {
    std::vector<double> width_height;

    // Vector containing bb width and height found from known object size and bb dimensions observed.
    width_height = (obj_params_.width > obj_params_.height) ? ((bb(2) > bb(3)) ? std::vector<double>{bb(2), bb(3)} : std::vector<double>{bb(3), bb(2)}) : ((bb(2) > bb(3)) ? std::vector<double>{bb(3), bb(2)} : std::vector<double>{bb(2), bb(3)});

    Eigen::Vector3d bb_center;
    bb_center << bb.segment(0, 2), 1;
    // Get a guess approximate size of object visible in the image. Assume that
    // the object is at the same height as camera. Average between size of the
    // smallest object side and the object oriented 45 deg.
    float expected_average_width = 0.5 *
                                   (COS_45 * (obj_params_.width + obj_params_.length) +
                                    std::min(obj_params_.width, obj_params_.length));

    double distance;
    distance = cam_params_.K.at<float>(0, 0) * expected_average_width /
               width_height[0];
    // Compensate for object center point and not closest face
    distance += obj_params_.length / 2;
    Eigen::Vector3d pos = cam_params_.K_inv * distance * bb_center;
    Eigen::Quaterniond quat;
    quat = Eigen::Quaterniond(Eigen::AngleAxisd(bb[4], Eigen::Vector3d::UnitZ()));

    pos = cam_params_.tf_ego_cam * pos;

    StateVec s;
    s << pos, 0, 0, 0, 0, 0, 0, quat.coeffs();
    return s;
  }

  // Used if bounding box is aligned
  // Approximates a full UKF state (6DoF pose) from a 2D bounding box measurement. Relies on heuristics and assumptions.
  template <bool is_aligned_bb = aligned_bb>
  StateVec approxStateFromBbHeuristic(
      typename std::enable_if<is_aligned_bb, const MeasureVec &>::type bb)
  {
    std::vector<double> width_height;

    // Vector containing bb width and height found from known object size and bb dimensions observed.
    width_height = (obj_params_.width > obj_params_.height) ? ((bb(2) > bb(3)) ? std::vector<double>{bb(2), bb(3)} : std::vector<double>{bb(3), bb(2)}) : ((bb(2) > bb(3)) ? std::vector<double>{bb(3), bb(2)} : std::vector<double>{bb(2), bb(3)});

    double d;
    Eigen::Vector3d bb_center;
    bb_center << bb.segment(0, 2), 1;
    // Get a guess approximate size of object visible in the image. Assume that
    // the object is at the same height as camera. Average between size of the
    // smallest object side and the object oriented 45 deg.
    float expected_average_width = 0.5 *
                                   (COS_45 * (obj_params_.width + obj_params_.length) +
                                    std::min(obj_params_.width, obj_params_.length));
    d = cam_params_.K.at<float>(0, 0) * expected_average_width /
        width_height[0];
    // Compensate for object center point and not closest face
    d += obj_params_.length / 2;
    Eigen::Vector3d pos = cam_params_.K_inv * d * bb_center;

    pos = cam_params_.tf_ego_cam * pos;

    StateVec s;
    s << pos, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    s.quat(0).setIdentity();
    return s;
  }

  // Used if bounding box is aligned
  template <bool is_aligned_bb = aligned_bb>
  double compareTwoBoundingBoxes(
      typename std::enable_if<is_aligned_bb, const MeasureVec &>::type bb1,
      typename std::enable_if<is_aligned_bb, const MeasureVec &>::type bb2)
  {
    // Score uses sum of square differences between bounding box center coordinates and square root of their sizes
    return 0.2 * pow(bb1[0] - bb2[0], 2) + 0.2 * pow(bb1[1] - bb2[1], 2) + pow(sqrt(bb1[2]) - sqrt(bb2[2]), 2) + pow(sqrt(bb1[3]) - sqrt(bb2[3]), 2);
  }

  // Used if bounding box is angled
  template <bool is_aligned_bb = aligned_bb>
  double compareTwoBoundingBoxes(
      typename std::enable_if<!is_aligned_bb, const MeasureVec &>::type bb1,
      typename std::enable_if<!is_aligned_bb, const MeasureVec &>::type bb2)
  {
    // TODO Include angle difference. Maybe use distances between each corner for angled boxes?
    // Score uses sum of square differences between bounding box center coordinates and square root of their sizes
    return 0.2 * pow(bb1[0] - bb2[0], 2) + 0.2 * pow(bb1[1] - bb2[1], 2) + pow(sqrt(bb1[2]) - sqrt(bb2[2]), 2) + pow(sqrt(bb1[3]) - sqrt(bb2[3]), 2);
  }

  /* measurement model definition */
  MeasureVec boundingBoxFromPose(const PoseVec &pose)
  {
    StateVec s = StateVec::Zero();
    s.head<3>() = pose.head<3>();
    s.tail<4>() = pose.tail<4>();
    return H(s);
  }

  PoseVec stateToPose(const StateVec &state_vec)
  {
    PoseVec pose;
    pose.head<3>() = state_vec.head<3>();
    pose.tail<4>() = state_vec.tail<4>();
    return pose;
  }

  StateVec poseToState(const PoseVec &pose_vec)
  {
    StateVec state = StateVec::Zero();
    state.head<3>() = pose_vec.head<3>();
    state.tail<4>() = pose_vec.tail<4>();
    return state;
  }

  std::tuple<double, StateVec> approxStatePoseOptimFromInit(const MeasureVec &target_bb, const StateVec &init_state)
  {
    PoseVec curr_pose, best_pose;
    curr_pose = stateToPose(init_state);
    best_pose = curr_pose;
    GradVec latest_update, grad;
    latest_update = GradVec::Zero();
    grad = GradVec::Zero();
    GradVec lr;
    lr = obj_params_.obj_pose_init_params.lr;

    // To keep track of errors and convergence
    double lowest_err, latest_err, curr_err;
    curr_err = compareTwoBoundingBoxes(target_bb, boundingBoxFromPose(curr_pose));
    lowest_err = curr_err;
    latest_err = curr_err;
    int nb_same_err = 0;

    int iter = 0;
    Eigen::Matrix<double, 7, 7> poses_steps_r, poses_steps_l, steps;
    MeasureVec bb_r, bb_l;

    while (nb_same_err < obj_params_.obj_pose_init_params.conv_steps && iter < obj_params_.obj_pose_init_params.max_steps)
    {

      grad.setZero();

      // Handle translation
      PoseVec l, r;

      // TODO Handle as matrix operation
      for (int i = 0; i < 3; i++)
      {
        l = curr_pose;
        r = curr_pose;
        l[i] -= obj_params_.obj_pose_init_params.grad_comp_step_sizes[i];
        r[i] += obj_params_.obj_pose_init_params.grad_comp_step_sizes[i];
        bb_l = boundingBoxFromPose(l);
        bb_r = boundingBoxFromPose(r);

        grad[i] = (compareTwoBoundingBoxes(target_bb, bb_r) - compareTwoBoundingBoxes(target_bb, bb_l)) / (2 * obj_params_.obj_pose_init_params.grad_comp_step_sizes[i]);
      }

      // Handle rotation and step along the quaternion with respect to the Euler axis
      Eigen::Quaterniond quat_l, quat_r, curr_quat;
      curr_quat = Eigen::Quaterniond(curr_pose[6], curr_pose[3], curr_pose[4], curr_pose[5]);

      for (int i = 3; i < 6; i++)
      {
        l = curr_pose;
        r = curr_pose;
        Eigen::Vector3d direction;
        switch (i)
        {
        case 3:
          direction = Eigen::Vector3d::UnitX();
          break;

        case 4:
          direction = Eigen::Vector3d::UnitY();
          break;

        case 5:
          direction = Eigen::Vector3d::UnitZ();
          break;
        }

        quat_l = Eigen::Quaterniond(Eigen::AngleAxisd(-obj_params_.obj_pose_init_params.grad_comp_step_sizes[i], direction));
        quat_r = Eigen::Quaterniond(Eigen::AngleAxisd(obj_params_.obj_pose_init_params.grad_comp_step_sizes[i], direction));

        l.tail<4>() = (quat_l * curr_quat).coeffs();
        r.tail<4>() = (quat_r * curr_quat).coeffs();

        bb_l = boundingBoxFromPose(l);
        bb_r = boundingBoxFromPose(r);

        grad[i] = (compareTwoBoundingBoxes(target_bb, bb_r) - compareTwoBoundingBoxes(target_bb, bb_l)) / (2 * obj_params_.obj_pose_init_params.grad_comp_step_sizes[i]);
      }

      latest_update = obj_params_.obj_pose_init_params.momentum * latest_update + (1 - obj_params_.obj_pose_init_params.momentum) * grad;
      curr_pose.head<3>() -= lr.head<3>().cwiseProduct(latest_update.head<3>());

      Eigen::Quaterniond rot_applied;
      rot_applied = Eigen::Quaterniond(Eigen::AngleAxisd(lr[3] * grad[3], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(lr[4] * grad[4], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(lr[5] * grad[5], Eigen::Vector3d::UnitZ()));

      curr_pose.tail<4>() = (rot_applied * curr_quat).coeffs();

      curr_err = compareTwoBoundingBoxes(target_bb, boundingBoxFromPose(curr_pose));

      if (curr_err < lowest_err)
      {
        lowest_err = curr_err;
        best_pose = curr_pose;
      }

      if (curr_err == latest_err)
      {
        nb_same_err++;
      }
      else
      {
        nb_same_err = 0;
      }

      if (iter % obj_params_.obj_pose_init_params.period_lower_lr == 0)
      {
        lr /= 2;
      }

      latest_err = curr_err;
      iter++;
    }
    StateVec out_state;
    out_state = poseToState(best_pose);
    return std::make_tuple(lowest_err, out_state);
  }

  std::tuple<double, StateVec> approxStatePoseOptim(const MeasureVec &target_bb)
  {
    std::vector<std::tuple<double, StateVec>> outputs;
    std::tuple<double, StateVec> output;

#pragma omp parallel for
    for (int i = 0; i < obj_params_.obj_pose_init_params.init_pose_guesses.size(); i++)
    {
      output = approxStatePoseOptimFromInit(target_bb, poseToState(obj_params_.obj_pose_init_params.init_pose_guesses[i]));
      outputs.push_back(output);
    }

    // Sort results by error, which is the first value of the returned tuple, stored in the vector.
    std::sort(outputs.begin(), outputs.end(),
              [](std::tuple<double, StateVec> const &lhs, std::tuple<double, StateVec> const &rhs)
              { return std::get<0>(lhs) < std::get<0>(rhs); });
    return outputs[0];
  }

  std::tuple<double, StateVec> approxStatePoseOptimSingleThread(const MeasureVec &target_bb)
  {
    std::vector<std::tuple<double, StateVec>> outputs;
    std::tuple<double, StateVec> output;

    for (PoseVec const &init_pose_guess : obj_params_.obj_pose_init_params.init_pose_guesses)
    {
      output = approxStatePoseOptimFromInit(target_bb, poseToState(init_pose_guess));
      outputs.push_back(output);
    }

    // Sort results by error, which is the first value of the returned tuple, stored in the vector.
    std::sort(outputs.begin(), outputs.end(),
              [](std::tuple<double, StateVec> const &lhs, std::tuple<double, StateVec> const &rhs)
              { return std::get<0>(lhs) < std::get<0>(rhs); });

    return outputs[0];
  }

  // Converts the state with pose and velocities to a 4x4 transformation matrix relating the pose to the origin.
  static void stateToTf(const StateVec &state, Eigen::Affine3f &tf)
  {
    tf = Eigen::Affine3f::Identity() *
         Eigen::Translation3f(Eigen::Vector3f(state(0), state(1), state(2))) *
         Eigen::Quaternionf(state.quat(0).w(),
                            state.quat(0).x(),
                            state.quat(0).y(),
                            state.quat(0).z());
  }

  // Used if bounding box is angled
  // Converts an OpenCV rotated rectangle to a measurement vector for the UKF
  template <bool is_aligned_bb = aligned_bb>
  MeasureVec convertRotRectToMeasureVec(
      typename std::enable_if<!is_aligned_bb, const cv::RotatedRect &>::type
          rect)
  {
    MeasureVec out;
    float angle, ar_meas;
    ar_meas = rect.size.width / rect.size.height;
    // Transform angle to wanted convention
    if ((ar_meas > 1 && obj_params_.aspect_ratio < 1) ||
        (ar_meas < 1 && obj_params_.aspect_ratio > 1))
    {
      angle = rect.angle + 90;
    }
    else
    {
      angle = rect.angle;
    }

    out << rect.center.x, rect.center.y, rect.size.width, rect.size.height,
        angle * M_PI / 180;
    return out;
  }

  // Used if bounding box is aligned
  // Converts an OpenCV rotated rectangle to a measurement vector for the UKF
  template <bool is_aligned_bb = aligned_bb>
  static MeasureVec convertRotRectToMeasureVec(
      typename std::enable_if<is_aligned_bb, const cv::RotatedRect &>::type
          rect)
  {
    cv::Rect2f upright_rect = rect.boundingRect2f();
    MeasureVec out;
    // Top left corner position given
    out << upright_rect.x + upright_rect.width / 2,
        upright_rect.y + upright_rect.height / 2, upright_rect.width,
        upright_rect.height;
    return out;
  }

  // Measurement model definition, based on projecting the pose into the image using camera parameters.
  MeasureVec H(const StateVec &state, const InputType &u = Eigen::Affine3d::Identity())
  {
    Eigen::Affine3f tf_ego_ado;
    stateToTf(state, tf_ego_ado);

    Eigen::MatrixXf model_points_cam;

    model_points_cam = ((cam_params_.tf_cam_ego * tf_ego_ado).matrix() *
                        obj_params_.model_points_ado_aug_t)
                           .template topRows<3>();
    model_points_cam.transposeInPlace();
    std::vector<cv::Point2f> projected_points;
    cv::Mat points_cv;
    cv::eigen2cv(model_points_cam, points_cv);

    cv::projectPoints(points_cv,
                      cv::Mat::zeros(3, 1, CV_32F),
                      cv::Mat::zeros(3, 1, CV_32F),
                      cam_params_.K,
                      cam_params_.dist_coeffs,
                      projected_points);

    MeasureVec out =
        convertRotRectToMeasureVec(cv::minAreaRect(projected_points));
    return out;
  }

  /* process model definition */
  /* Applied before the propagation using dF (defined above) and the
   * integrators. Updates that rely on integrations of the state should be
   * defined on the dF function. Here is good for changing the frame before
   * integration. */
  StateVec G(const StateVec &state, const InputType &tf_ego_egoprev)
  {
    StateVec out;

    // Position
    out.head<3>() = tf_ego_egoprev * state.head<3>();
    // Linear velocity
    out.segment<3>(3) = tf_ego_egoprev.linear() * state.segment<3>(3);
    // Angular velocity
    out.segment<3>(6) = tf_ego_egoprev.linear() * state.segment<3>(6);
    // Quaternion representation of orientation
    out.quat(0) = Eigen::Quaterniond(tf_ego_egoprev.linear() * state.quat(0));
    return out;
  }

  void updateCamera(msl_raptor_backend::CameraParams cam_params)
  {
    cam_params_ = cam_params;
  }

  void updateCamIntrinsics(const float ppx,
                           const float ppy,
                           const float fx,
                           const float fy,
                           const cv::Mat &dist_coeffs_in)
  {
    cam_params_.updateIntrinsics(ppx, ppy, fx, fy, dist_coeffs_in);
  }

  void updateCamIntrinsics(const std::vector<float> &camera_matrix_in,
                           const std::vector<float> &dist_coeffs_in)
  {
    cam_params_.updateIntrinsics(camera_matrix_in, dist_coeffs_in);
  }

  void updateCamIntrinsics(const cv::Mat &camera_matrix_in,
                           const cv::Mat &dist_coeffs_in)
  {
    cam_params_.updateIntrinsics(camera_matrix_in, dist_coeffs_in);
  }

  void updateCamExtrinsics(const Eigen::Affine3d &extrinsics_mat)
  {
    cam_params_.updateExtrinsics(extrinsics_mat);
  }

  void updateCamExtrinsics(const cv::Mat &rvec_in, const cv::Mat &tvec_in)
  {
    cam_params_.updateExtrinsics(rvec_in, tvec_in);
  }

  void updateCamExtrinsics(const std::vector<float> &rvec_in,
                           const std::vector<float> &tvec_in)
  {
    cam_params_.updateExtrinsics(rvec_in, tvec_in);
  }

  // Update the UKF
  void update(double delta_t,
              const MeasureVec &z,
              const InputType &input = Eigen::Affine3d::Identity())
  {
    ukf_->update(delta_t, z, input);
  }

  StateVec getState()
  {
    return ukf_->state;
  }

  Eigen::MatrixXd getStateRootCov()
  {
    return ukf_->stateRootCov;
  }

  // Initialise the MSL-RAPTOR UKF based on object, camera parameters, and an initial UKF state.
  void init(msl_raptor_backend::ObjParams obj_params,
            msl_raptor_backend::CameraParams cam_params,
            StateVec init_state)
  {
    obj_params_ = obj_params;
    cam_params_ = cam_params;
    ukf_ = new kalman::UKF<MSLRaptorUKF, InputType>(*this);
    ukf_->state = init_state;
  }

  MSLRaptorUKF(void) {}

  MSLRaptorUKF(msl_raptor_backend::ObjParams obj_params,
               msl_raptor_backend::CameraParams cam_params)
  {
    StateVec init_state = StateVec::Zero();
    init_state.quat(0).setIdentity();
    init(obj_params, cam_params, init_state);
  }

  // Initialise the MSL-RAPTOR UKF based on object, camera parameters, and an initial UKF state.
  MSLRaptorUKF(msl_raptor_backend::ObjParams obj_params,
               msl_raptor_backend::CameraParams cam_params,
               StateVec init_state)
  {
    init(obj_params, cam_params, init_state);
  }

  // Initialise the MSL-RAPTOR UKF based on object, camera parameters, and a 2D bb from which to approximate a UKF state.
  MSLRaptorUKF(msl_raptor_backend::ObjParams obj_params,
               msl_raptor_backend::CameraParams cam_params,
               MeasureVec bb_for_approx_init)
  {
    obj_params_ = obj_params;
    cam_params_ = cam_params;
    StateVec init_state = approxStateFromBbHeuristic(bb_for_approx_init);
    init(obj_params, cam_params, init_state);
  }

private:
  msl_raptor_backend::ObjParams obj_params_;
  msl_raptor_backend::CameraParams cam_params_;
  kalman::UKF<MSLRaptorUKF, InputType> *ukf_;
};
// ------------------------------------------------------------------------------------
// ----------------------- END MSL-RAPTOR Back-end UKF definition -------------------
// ------------------------------------------------------------------------------------