#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<rover_arm_moveit_parser/ArmJointStates.h>
#include<math.h>

joint_parser::ArmJointStates arm_trajectory;
joint_parser::ArmJointStates total;

int stepsPerRevolution[6] = {1000,1000,1000,1000,1000,1000}; //microsteps/revolution (using 16ths) from observation, for each motor


int joint_status = 0; 
double cur_angle[6]; //
int joint_step[6]; //
double prev_angle[6] = {0,0,0,0,0,0}; //
double init_angle[6] = {0,0,0,0,0,0}; //
double total_steps[6] = {0,0,0,0,0,0}; //track number of steps
int count = 0;

void parse_joints(const sensor_msgs::JointState& arm){
  total.base_roll = arm.position[0];
  total.shoulder_pitch = arm.position[1];
  total.elbow_pitch = arm.position[2];
  total.elbow_roll = arm.position[3];
  total.wrist_pitch = arm.position[4];
  total.wrist_roll  = arm.position[5];
   /*
  if (count==0){
    prev_angle[0] = arm.position[0];
    prev_angle[1] = arm.position[1];
    prev_angle[2] = arm.position[2];
    prev_angle[3] = arm.position[3];
    prev_angle[4] = arm.position[4];
    prev_angle[5] = arm.position[5];

    init_angle[0] = arm.position[0];
    init_angle[1] = arm.position[1];
    init_angle[2] = arm.position[2];
    init_angle[3] = arm.position[3];
    init_angle[4] = arm.position[4];
    init_angle[5] = arm.position[5];
  }

  ROS_INFO_STREAM("Received /joint_states");

  
  arm_trajectory.base_roll = (int)((arm.position[0]-prev_angle[0])*stepsPerRevolution[0]/(2*M_PI));
  arm_trajectory.shoulder_pitch = (int)((arm.position[1]-prev_angle[1])*stepsPerRevolution[1]/(2*M_PI));
  arm_trajectory.elbow_pitch = (int)((arm.position[2]-prev_angle[2])*stepsPerRevolution[2]/(2*M_PI));
  arm_trajectory.elbow_roll = (int)((arm.position[3]-prev_angle[3])*stepsPerRevolution[3]/(2*M_PI));
  arm_trajectory.wrist_pitch = (int)((arm.position[4]-prev_angle[4])*stepsPerRevolution[4]/(2*M_PI));
  arm_trajectory.wrist_roll = (int)((arm.position[5]-prev_angle[5])*stepsPerRevolution[5]/(2*M_PI));

  if (count!=0){
    prev_angle[0] = arm.position[0];
    prev_angle[1] = arm.position[1];
    prev_angle[2] = arm.position[2];
    prev_angle[3] = arm.position[3];
    prev_angle[4] = arm.position[4];
    prev_angle[5] = arm.position[5];
  }

  total.base_roll += arm_trajectory.base_roll;
  total.shoulder_pitch += arm_trajectory.shoulder_pitch;
  total.elbow_pitch += arm_trajectory.elbow_pitch;
  total.elbow_roll += arm_trajectory.elbow_roll;
  total.wrist_pitch += arm_trajectory.wrist_pitch;
  total.wrist_roll += arm_trajectory.wrist_roll; 

  joint_status = 1;
  count=1;*/
}

int main(int argc, char **argv){
   ros::init(argc, argv,"joint_parser");
   ros::NodeHandle nh;
   ros::Subscriber sub = nh.subscribe("/joint_states", 1000, parse_joints);
   ros::Publisher pub = nh.advertise<rover_arm_moveit_parser::ArmJointStates>("joint_states_remap",50);
  
   ros::Rate loop_rate(20);
   
   while (ros::ok()){
      if(joint_status==1){
   	 joint_status = 0;
   	 pub.publish(total);      
	 ROS_INFO_STREAM("Published to /joint_states_remap");
      }
      ros::spinOnce();
      loop_rate.sleep();  
   }
  return 0;
}
