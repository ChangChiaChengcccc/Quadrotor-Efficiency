#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Accel.h>
#include <Eigen/Dense>
#include <vector>
#include <std_msgs/Float64MultiArray.h>

using namespace std;
using namespace Eigen;

float rate = 40;
float dt = 1/rate;
float x,y,z,w;
float m=1.5,g=9.8;
float f_f;
float average;

Matrix3d R;
Vector3d e_3(0,0,1);
Vector3d imu_accel;
Matrix3d J;
Vector3d imu_omega(0,0,0);
Vector3d imu_omega_dot;
Vector3d imu_omega_pass(0,0,0);
Vector3d imu_omega_filter(0,0,0);
Vector3d accel;
Vector3d M_f;
Vector4d fault_control_input;
geometry_msgs::Accel test_accel;
Matrix4d allocation_matrix,allocation_matrix_tmp,allocation_inv;
Vector4d control_input,control_input_tmp;
double rotor_force_constant = 8.54858e-6;

std_msgs::Float64MultiArray msg;

// debug
Array3d right_side,left_side,ratio;
Array4d right_hand,left_hand;
Array4d E(1,1,1,1),F;

geometry_msgs::Pose current_pos;
void pose_cb(const geometry_msgs::Pose::ConstPtr& msg)
{
  current_pos = *msg;
  // add test publish

}

sensor_msgs::Imu current_imu;
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
  current_imu = *msg;
}
Vector4d current_control_input_material;
void control_input_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	float dstride0 = msg->layout.dim[0].stride;
	float dstride1 = msg->layout.dim[1].stride;
	float h = msg->layout.dim[0].size;
	float w = msg->layout.dim[1].size;

	std::vector<double> data = msg->data;
	Eigen::Vector4d vec;
  for(int i=0; i<4; i++){
    vec[i] = data[i];
  }
  current_control_input_material = vec;
	//std::cout << "I received = " << std::endl << current_allocation_material << std::endl;
}

Matrix4d current_allocation_material;
void allocation_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	float dstride0 = msg->layout.dim[0].stride;
	float dstride1 = msg->layout.dim[1].stride;
	float h = msg->layout.dim[0].size;
	float w = msg->layout.dim[1].size;

	// Below are a few basic Eigen demos:
	std::vector<double> data = msg->data;
	Eigen::Map<Eigen::MatrixXd> mat(data.data(), h, w);
  current_allocation_material = mat;
	//std::cout << "I received = " << std::endl << current_allocation_material << std::endl;
}


int main(int argc, char **argv)
{
  //test
  std::vector<double> vec1 = { E(0), E(1), E(2), E(3), 0.81};
  // set up dimensions
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = vec1.size();
  msg.layout.dim[0].stride = 1;
  msg.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1

  // copy in the data
  msg.data.clear();
  msg.data.insert(msg.data.end(), vec1.begin(), vec1.end());

  ros::init(argc, argv, "get_efficiency_node");
  ros::NodeHandle n;
  //ros::Rate loop_rate(rate);
  ros::Subscriber pose_sub = n.subscribe("iris1/ground_truth/pose", 10, pose_cb);
  ros::Subscriber imu_sub = n.subscribe("iris1/ground_truth/imu", 10, imu_cb);
  ros::Subscriber control_input_sub = n.subscribe("control_input", 10, control_input_cb);
  ros::Subscriber allocation_sub = n.subscribe("allocation", 10, allocation_cb);
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("efficiency", 1);
  ros::Rate loop_rate(rate);

  while(ros::ok()){
    // get the quaternion
    x = current_pos.orientation.x;
    y = current_pos.orientation.y;
    z = current_pos.orientation.z;
    w = current_pos.orientation.w;

    // get the acceleration (body-fixed frame)
    imu_accel[0] = current_imu.linear_acceleration.x;
    imu_accel[1] = current_imu.linear_acceleration.y;
    imu_accel[2] = current_imu.linear_acceleration.z;

    // get the angular acceleration (body-fixed frame)
    imu_omega_filter[0] = 0.97*imu_omega_pass[0] + 0.03*current_imu.angular_velocity.x;
    imu_omega_filter[1] = 0.97*imu_omega_pass[1] + 0.03*current_imu.angular_velocity.y;
    imu_omega_filter[2] = 0.97*imu_omega_pass[2] + 0.03*current_imu.angular_velocity.z;

    imu_omega_dot[0] = imu_omega_filter[0];
    imu_omega_dot[1] = imu_omega_filter[1];
    imu_omega_dot[2] = imu_omega_filter[2];

    imu_omega_pass[0] = imu_omega_filter[0];
    imu_omega_pass[1] = imu_omega_filter[1];
    imu_omega_pass[2] = imu_omega_filter[2];

    // imu_omega_dot[0] = (current_imu.angular_velocity.x - imu_omega[0])/dt;
    // imu_omega_dot[1] = (current_imu.angular_velocity.x - imu_omega[1])/dt;
    // imu_omega_dot[2] = (current_imu.angular_velocity.x - imu_omega[2])/dt;
    //cout << "imu_omega_dot: \n" << imu_omega_dot << endl;

    // get the angular velocity (body-fixed frame)
    imu_omega[0] = current_imu.angular_velocity.x;
    imu_omega[1] = current_imu.angular_velocity.y;
    imu_omega[2] = current_imu.angular_velocity.z;
    //cout << "imu_omega:\n"<<imu_omega << endl;

    // transfer quaternion to rotation matrix
    R << 1-2*(y*y+z*z),   2*(x*y-w*z),  2*(w*y+x*z),
          2*(x*y+w*z), 1-2*(x*x+z*z),  2*(y*z-w*x),
          2*(x*z-w*y),   2*(y*z+w*x),1-2*(x*x+y*y); 
    //cout << "rotation matrix: \n" << R << endl;

    // calculate the acceleration (inertial frame)
    accel = R*imu_accel;
    // to get the v_dot in geometry paper
    accel[2] = accel[2] -9.8;
    //cout << "imu_accel:\n" << imu_accel << endl;
    //cout << "accel:\n" << accel << endl;

    // get the control_input(f M1 M2 M3)
    control_input_tmp = current_control_input_material;
    control_input[0] = control_input_tmp[3];
    control_input[1] = control_input_tmp[0];
    control_input[2] = control_input_tmp[1];
    control_input[3] = control_input_tmp[2];
    //cout << "control_input: \n" << control_input << endl;

    // get the allocation matrix L
    allocation_matrix_tmp = current_allocation_material.transpose()*(current_allocation_material*current_allocation_material.transpose()).inverse();
    allocation_matrix_tmp = (1/rotor_force_constant)*allocation_matrix_tmp;
    allocation_matrix << allocation_matrix_tmp(3),allocation_matrix_tmp(7),allocation_matrix_tmp(11),allocation_matrix_tmp(15),
                         allocation_matrix_tmp(0),allocation_matrix_tmp(4),allocation_matrix_tmp(8) ,allocation_matrix_tmp(12),
                         allocation_matrix_tmp(1),allocation_matrix_tmp(5),allocation_matrix_tmp(9) ,allocation_matrix_tmp(13),
                         allocation_matrix_tmp(2),allocation_matrix_tmp(6),allocation_matrix_tmp(10),allocation_matrix_tmp(14);
    //cout << "allocation_matrix: \n" << allocation_matrix << endl;

    // Assuming that the faults happens, get f_f and M_f 
    // f_f
    left_side = R*e_3;
    right_side = m*g*e_3 + m*accel;
    ratio = right_side / left_side;
    // cout << "R*e3:\n" << left_side << endl <<
    //         "m*g*e_3 + m*accel:\n" << right_side << endl <<
    //         "2/1:\n" << ratio << endl << endl;
    f_f = ratio[2];
    

    // M_f
    J << 0.0347563,0,0,
         0,0.0458929,0,
         0,0,0.0977;
    
    M_f = J*imu_omega_dot+imu_omega.cross(J*imu_omega);
    //cout << "M_f:\n" << M_f << endl;

    // E_diag
    fault_control_input << f_f,
                           M_f;
    cout << "fault_control_input:\n" << fault_control_input << endl;
    left_hand = allocation_matrix.inverse()*fault_control_input;
    allocation_inv = allocation_matrix.transpose() * (allocation_matrix*allocation_matrix.transpose()).inverse();
    left_hand = allocation_inv*fault_control_input;
    F = allocation_inv*control_input;
    E = left_hand/F;
    cout << "E:\n" << E << endl;
    vec1 = { E(0), E(1), E(2), E(3), 0.81};
    msg.data.clear();
    msg.data.insert(msg.data.end(), vec1.begin(), vec1.end());
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
