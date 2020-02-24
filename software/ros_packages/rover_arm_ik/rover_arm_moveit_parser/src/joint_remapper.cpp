#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<rover_arm_moveit_parser/ArmJointStates.h>
#include<math.h>

rover_arm_moveit_parser::ArmJointStates arm_steps;
rover_arm_moveit_parser::ArmJointStates total;

int stepsPerRevolution[6] = {32800,18000,72000,3280,14400,0};  //microsteps/revolution (using 16ths) from observation, for each motor
//TODO: check values for stepsPerRevolution

int joint_status = 0; 
double cur_angle[6]; //
int joint_step[6]; //
double prev_angle[6] = {0,0,0,0,0,0}; //
double init_angle[6] = {0,0,0,0,0,0}; //
double total_steps[6] = {0,0,0,0,0,0}; //track number of steps
int count = 0;

void parse_joints(const sensor_msgs::JointState& arm){
//rosmsg show sensor_msg/JointState for info
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

  ROS_INFO_STREAM("Received /move_group/fake_controller_joint_states");

  
  arm_steps.base = (int)((arm.position[0]-prev_angle[0])*stepsPerRevolution[0]/(2*M_PI));
  arm_steps.shoulder = (int)((arm.position[1]-prev_angle[1])*stepsPerRevolution[1]/(2*M_PI));
  arm_steps.elbow = (int)((arm.position[2]-prev_angle[2])*stepsPerRevolution[2]/(2*M_PI));
  arm_steps.roll = (int)((arm.position[3]-prev_angle[3])*stepsPerRevolution[3]/(2*M_PI));
  arm_steps.wrist_pitch = (int)((arm.position[4]-prev_angle[4])*stepsPerRevolution[4]/(2*M_PI));
  arm_steps.wrist_roll = (int)((arm.position[5]-prev_angle[5])*stepsPerRevolution[5]/(2*M_PI));

  ROS_INFO_NAMED("test", "arm_steps.wrist_pitch #2: %d", arm_steps.wrist_pitch);

  if (count!=0){
    prev_angle[0] = arm.position[0];
    prev_angle[1] = arm.position[1];
    prev_angle[2] = arm.position[2];
    prev_angle[3] = arm.position[3];
    prev_angle[4] = arm.position[4];
    prev_angle[5] = arm.position[5];
  }

  total.base += arm_steps.base;
  total.shoulder += arm_steps.shoulder;
  total.elbow += arm_steps.elbow;
  total.roll += arm_steps.roll;
  total.wrist_pitch += arm_steps.wrist_pitch;
  total.wrist_roll += arm_steps.wrist_roll; 

  ROS_INFO_NAMED("test", "total_steps[4]: %f, total: %d", total_steps[4], total.wrist_pitch);
  ROS_INFO_NAMED("test", "arm_steps.wrist_pitch #3: %d", arm_steps.wrist_pitch);
  ROS_INFO_STREAM("Done conversion to /joint_steps");
  joint_status = 1;
  count=1;
}

int main(int argc, char **argv){
   ros::init(argc, argv,"joint_parser");
   ros::NodeHandle nh;
   //ROS_INFO_STREAM("In main function");
   ros::Subscriber sub = nh.subscribe("/joint_states",1000,parse_joints);
   ros::Publisher pub = nh.advertise<rover_arm_moveit_parser::ArmJointStates>("joint_steps",50);
  
   ros::Rate loop_rate(20);
   
   while (ros::ok()){
      if(joint_status==1){
      joint_status = 0;
      pub.publish(total);
      ROS_INFO_STREAM("Published to /joint_steps");
      }
      ros::spinOnce();
      loop_rate.sleep();  
   }
  return 0;
}
