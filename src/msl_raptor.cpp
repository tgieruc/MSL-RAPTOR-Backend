#include "msl_raptor_backend.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <Eigen/Core>
#include <chrono>
#include <iostream>

#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <angledbox_msgs/AngledBox.h>
#include <angledbox_msgs/AngledBoxArray.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <csignal>
const bool aligned_bb = false;                                  // Set the type of bounding box used, here angled

void mySigintHandler(int sig);

struct Drone{
    int id;
    MSLRaptorUKF<aligned_bb>::MeasureVec angled_box;
    MSLRaptorUKF<aligned_bb> msl_raptor_ukf;
    Eigen::MatrixXd msl_raptor_state;
    int updated = false;
};



class MslRaptorNode{
public:
    MslRaptorNode(ros::NodeHandle *nh);
    void spin(ros::NodeHandle *nh);

private:
    ros::Publisher pose_pub;
    ros::Publisher tf_pub;
    ros::Publisher time_pub;
    ros::Subscriber angled_box_sub;
    msl_raptor_backend::CameraParams cam_params;
    msl_raptor_backend::ObjParams obj_params;
    bool new_box = false;
    bool verbose = false;

    std::vector<Drone> drones;
    std::chrono::time_point<std::chrono::steady_clock> last_box_time;
    void angledBoxCallback(const angledbox_msgs::AngledBoxArray::ConstPtr& msg);
    void setObjParams(ros::NodeHandle *nh);
    void setCameraParams(ros::NodeHandle *nh);
};

MslRaptorNode::MslRaptorNode(ros::NodeHandle *nh){
    ros::Subscriber subscriber =  nh->subscribe("multi_tracker/angledbox_array", 10, &MslRaptorNode::angledBoxCallback, this);
    nh->getParam("msl_raptor/verbose", this->verbose);

    this->setCameraParams(nh);
    this->setObjParams(nh);
    while (!this->new_box){
        ros::spinOnce();
    }
    this->new_box = false;
    this->last_box_time = std::chrono::steady_clock::now();
}

void MslRaptorNode::setCameraParams(ros::NodeHandle *nh) {
    std::vector<float> cam_mat; //camera matrix, extrinsics, distorsion coefficients)
    std::vector<float> rvec;
    std::vector<float> tvec;
    std::vector<float> dist_coeffs;

    nh->getParam("msl_raptor/camera_matrix", cam_mat);
    nh->getParam("msl_raptor/rvec", rvec);
    nh->getParam("msl_raptor/tvec", tvec);
    nh->getParam("msl_raptor/dist_coeffs", dist_coeffs);
    this->cam_params.init(cam_mat, rvec, tvec, dist_coeffs);
}

void MslRaptorNode::setObjParams(ros::NodeHandle *nh) {
    std::vector<double> sigma;
    std::vector<double> process_noise;
    std::vector<double> meas_noise;
    std::vector<double> object_whl;
    double size_scale;

    nh->getParam("msl_raptor/sigma", sigma);
    nh->getParam("msl_raptor/process_noise", process_noise);
    nh->getParam("msl_raptor/meas_noise", meas_noise);
    nh->getParam("msl_raptor/object_whl", object_whl);
    nh->getParam("msl_raptor/size_scale", size_scale);

    // Parameters for optimisation-based pose estimation from bb
    double momentum;                                         // Momentum parameter
    int max_steps;                                           // Maximum number of steps
    int conv_steps;                                            // Steps to check convergence
    int period_lower_lr;                                      // How often to reduce learning rate

    nh->getParam("msl_raptor/momentum", momentum);
    nh->getParam("msl_raptor/max_steps", max_steps);
    nh->getParam("msl_raptor/conv_steps", conv_steps);
    nh->getParam("msl_raptor/period_lower_lr", period_lower_lr);

    std::vector<msl_raptor_backend::PoseVec> bb_init_pose_guesses; // Contains position vector and quaternion for orientation
    // Add a grid of initial guesses
    for (int i = -2; i < 2; i ++)
    {
        for (int j = -2; j < 2; j ++)
        {
            for (int k = 1; k < 8; k ++)
            {
                Eigen::Matrix<double, 7, 1> a {double(i), double(j), double(k), 0., 0., 0., 1.};
                bb_init_pose_guesses.push_back(a); // Position vector and orientation quaternion
            }
        }
    }
    std::vector<double> bb_init_step_size;    // Step sizes along position and Euler axis for orientation
    std::vector<double> bb_init_lr; // Step sizes for gradient approximation along position and Euler axis for orientation

    nh->getParam("msl_raptor/bb_init_step_size", bb_init_step_size);
    nh->getParam("msl_raptor/bb_init_lr", bb_init_lr);

    msl_raptor_backend::ObjPoseInitParams obj_pose_init_params(bb_init_step_size, bb_init_lr, bb_init_pose_guesses, momentum, max_steps, conv_steps, period_lower_lr);

    // Provides width, height and length of object in meters, from which corners of a 3D box are used to approximate the object's shape.
    this->obj_params = msl_raptor_backend::ObjParams(size_scale * object_whl[0], size_scale * object_whl[1], size_scale * object_whl[2], sigma, process_noise, meas_noise, obj_pose_init_params);
}


void MslRaptorNode::spin(ros::NodeHandle *nh) {
    ros::Subscriber subscriber =  nh->subscribe("multi_tracker/angledbox_array", 10, &MslRaptorNode::angledBoxCallback, this);
    this->pose_pub = nh->advertise<geometry_msgs::PoseArray>("msl_raptor/pose", 1000);
    this->tf_pub = nh->advertise<tf2_msgs::TFMessage>("tf", 1000);
    this->time_pub = nh->advertise<std_msgs::Float32>("msl_raptor/time", 1000);

    while (ros::ok()) {
        // wait for new bounding box
        while (!this->new_box){
            ros::spinOnce();
        }
        this->new_box = false;
        std::chrono::time_point<std::chrono::steady_clock> t0 = std::chrono::steady_clock::now();
        std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
        std::chrono::duration<double> dt = now - this->last_box_time;
        this->last_box_time = now;

        //Update states
        geometry_msgs::PoseArray pose_array;
        tf2_msgs::TFMessage  tf_array;
        pose_array.header.frame_id = "map";

        for (auto & drone : this->drones){
            if (drone.updated) {
                drone.msl_raptor_ukf.update(dt.count(), drone.angled_box);
                drone.msl_raptor_state = drone.msl_raptor_ukf.getState();
            }
            if (this->verbose == true){
                std::cout << "The new pose of id " << drone.id << " is " << std::endl
                          << drone.msl_raptor_ukf.stateToPose(drone.msl_raptor_state) << "\n\n";
            }


            geometry_msgs::TransformStamped transform;
            geometry_msgs::Pose pose;
            transform.child_frame_id = "msl_raptor";
            transform.transform.translation.x = drone.msl_raptor_state(0);
            transform.transform.translation.y = drone.msl_raptor_state(1);
            transform.transform.translation.z = drone.msl_raptor_state(2);
            transform.transform.rotation.x = drone.msl_raptor_state(8);
            transform.transform.rotation.y = drone.msl_raptor_state(9);
            transform.transform.rotation.z = drone.msl_raptor_state(10);
            transform.transform.rotation.w = drone.msl_raptor_state(11);
            tf_array.transforms.push_back(transform);

            pose.position.x    = drone.msl_raptor_state(0);
            pose.position.y    = drone.msl_raptor_state(1);
            pose.position.z    = drone.msl_raptor_state(2);
            pose.orientation.x = drone.msl_raptor_state(8);
            pose.orientation.y = drone.msl_raptor_state(9);
            pose.orientation.z = drone.msl_raptor_state(10);
            pose.orientation.w = drone.msl_raptor_state(11);
            pose_array.poses.push_back(pose);
        }
        now = std::chrono::steady_clock::now();
        std::chrono::duration<double> inf_time = now - t0;

        float inf_time_f = inf_time.count();
        std_msgs::Float32 inf_time_ros;
        inf_time_ros.data = inf_time_f;
        this->time_pub.publish(inf_time_ros);
        this->pose_pub.publish(pose_array);
        this->tf_pub.publish(tf_array);
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "msl_raptor_node");
    ros::NodeHandle nh;
    MslRaptorNode node(&nh);
    signal(SIGINT, mySigintHandler);
    node.spin(&nh);
}

void mySigintHandler(int sig){
    exit(0);
}


void MslRaptorNode::angledBoxCallback(const angledbox_msgs::AngledBoxArray::ConstPtr& msg){
    std::vector<angledbox_msgs::AngledBox> angled_box_vec;
    angled_box_vec.assign(std::begin(msg->angledbox_array), std::end(msg->angledbox_array));
    std::vector<int> drone_id;
    for (auto & drone_msg : angled_box_vec){
        bool found = false;
        drone_id.push_back(drone_msg.id);
        for (auto & drone : this->drones){
            if (drone_msg.id == drone.id){
                drone.angled_box << drone_msg.angledbox[0],
                                    drone_msg.angledbox[1],
                                    drone_msg.angledbox[2],
                                    drone_msg.angledbox[3],
                                    drone_msg.angledbox[4];
                drone.updated = true;
                found = true;
            }
        }

        if (!found){
            Drone drone;
            drone.id = drone_msg.id;
            drone.angled_box << drone_msg.angledbox[0],
                                drone_msg.angledbox[1],
                                drone_msg.angledbox[2],
                                drone_msg.angledbox[3],
                                drone_msg.angledbox[4];
            drone.msl_raptor_ukf = MSLRaptorUKF<aligned_bb>(this->obj_params, this->cam_params, drone.angled_box);
            drone.updated = true;
            this->drones.push_back(drone);
        }
    }

    auto it = this->drones.begin();
    while(it != this->drones.end()) {
        // If element is even number then delete it
        if(!std::count(drone_id.begin(), drone_id.end(), it->id)) {
            // Due to deletion in loop, iterator became
            // invalidated. So reset the iterator to next item.
            it = this->drones.erase(it);
        } else {
            it++;
        }
    }


    this->new_box = true;
}