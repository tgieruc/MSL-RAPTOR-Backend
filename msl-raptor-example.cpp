#include "msl_raptor_backend.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <type_traits>

int main(int argc, char **argv)
{
    // Define if we want to used an aligned bb (4 parameters) or an angled one (5 parameters).
    const bool aligned_bb = false;

    // Define camera parameters (camera matrix, extrinsics, distorsion coefficients) and create camera params object.
    std::vector<float> cam_mat{304.7262878417969, 0.0, 214.2415313720703,
                               0.0, 304.7262878417969, 121.70726013183594,
                               0.0, 0.0, 1.0};
    std::vector<float> rvec(3, 0.0);
    std::vector<float> tvec(3, 0.0);
    std::vector<float> dist_coeffs(4, 0);
    msl_raptor_backend::CameraParams cam_params(cam_mat, rvec, tvec, dist_coeffs);

    // Define object parameters (UKF initial covariances, process noise, and measurement noise, dimensions) and create object params object.
    std::vector<double> sigma{0.0000000000001, 0.000001, 0.000001,  // position
                              0.00000000000005, 0.000005, 0.000005, //  linear velocity
                              0.000003, 0.000003, 0.000003,         // angular velocity
                              0.000005, 0.000005, 0.00005};         // orientation

    std::vector<double> process_noise{0.002, 0.0002, 0.0002,     // position
                                      0.0001, 0.0000001, 0.0001, // linear velocity
                                      0.0006, 0.000006, 0.00006, // angular velocity
                                      0.00001, 0.0001, 0.00001}; // orientation

    // measurement vector for angle bb is center x, center y, width, height, angle.
    std::vector<double> meas_noise{1, 1, 2, 1, 0.5};

    // Parameters for optimisation-based pose estimation from bb
    double momentum = 0.9;                                         // Momentum parameter
    int max_steps = 500;                                           // Maximum number of steps
    int conv_steps = 4;                                            // Steps to check convergence
    int period_lower_lr = 20;                                      // How often to reduce learning rate
    std::vector<msl_raptor_backend::PoseVec> bb_init_pose_guesses; // Contains position vector and quaternion for orientation
    // Add a grid of initial guesses
    for (double i = -2; i < 2; i += 1)
    {
        for (double j = -2; j < 2; j += 1)
        {
            for (double k = 2; k < 8; k += 1)
            {
                bb_init_pose_guesses.push_back(msl_raptor_backend::PoseVec(i, j, k, 0, 0, 0, 1)); // Position vector and orientation quaternion
                // std::cout << msl_raptor_backend::PoseVec(i, j, k, 0, 0, 0, 1) << std::endl;
            }
        }
    }
    std::vector<double> bb_init_step_size{0.001, 0.001, 0.01, 0.001, 0.001, 0.001};    // Step sizes along position and Euler axis for orientation
    std::vector<double> bb_init_lr{0.0005, 0.0005, 0.0001, 0.00005, 0.00005, 0.00005}; // Step sizes for gradient approximation along position and Euler axis for orientation
    msl_raptor_backend::ObjPoseInitParams obj_pose_init_params(bb_init_step_size, bb_init_lr, bb_init_pose_guesses, momentum, max_steps, conv_steps, period_lower_lr);

    // Provides width, height and length of object in meters, from which corners of a 3D box are used to approximate the object's shape.
    msl_raptor_backend::ObjParams obj_params(0.3, 0.2, 0.5, sigma, process_noise, meas_noise, obj_pose_init_params);

    // Initial state, here places the object 5 meters away in front with no velocity.
    MSLRaptorUKF<aligned_bb>::StateVec true_state, state_approx_heuristic, state_approx_optim;
    true_state << 0, 0, 5.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    true_state.quat(0).setIdentity();

    // Initialize MSL-RAPTOR back-end UKF using set object parameters, camera parameters, and true state.
    MSLRaptorUKF<aligned_bb> msl_raptor_ukf(obj_params, cam_params, true_state);
    std::cout << "True pose is " << std::endl
              << msl_raptor_ukf.stateToPose(true_state) << std::endl
              << std::endl;

    // Check the measurement process applied to the current UKF state with no input, which returns a bounding box prediction.
    MSLRaptorUKF<aligned_bb>::MeasureVec m = msl_raptor_ukf.H(true_state, Eigen::Affine3d());

    std::cout << "Measurement at true state" << std::endl
              << m << std::endl
              << std::endl;

    // Example of approximating a state from a 2D bounding box with heuristics
    state_approx_heuristic = msl_raptor_ukf.approxStateFromBbHeuristic(m);
    std::cout << "Pose approximated only from 2D bounding box with a heuristic " << std::endl
              << msl_raptor_ukf.stateToPose(state_approx_heuristic) << std::endl
              << std::endl;

    // Example of approximating a state from a 2D bounding box with optimisation
    double optim_error;
    std::tie(optim_error, state_approx_optim) = msl_raptor_ukf.approxStatePoseOptim(m);
    std::cout << "Pose approximated only from 2D bounding box with optimisation (err " << optim_error << ") "
              << std::endl
              << msl_raptor_ukf.stateToPose(state_approx_optim) << std::endl
              << std::endl;

    // Modify the measurement width to make it appear closer
    m(2) *= 4;
    m(3) *= 4;
    std::cout << "What if we observed a bounding box with a larger size. This should indicate a closer object." << std::endl
              << m << std::endl
              << std::endl;

    // Update the UKF with the modified measurement, which should make the pose appear ...
    msl_raptor_ukf.update(0.03, m);

    // Check the new state
    std::cout << "Predicted pose after measurement update is " << std::endl
              << msl_raptor_ukf.stateToPose(msl_raptor_ukf.getState()) << "\n\n";

    // Create a fake input (could be from odometry in practice) 
    Eigen::Affine3d input = Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ()) *
                            Eigen::Translation3d(0.0, 0.0, 0.2);
    std::cout << "What if we apply a translation and orientation input with the measurement \n \n";

    // Update the UKF with the modified measurement and fake input
    msl_raptor_ukf.update(0.03, m, input);

    // Check the new state
    std::cout << "The new pose is " << std::endl
              << msl_raptor_ukf.stateToPose(msl_raptor_ukf.getState()) << "\n\n";
}
