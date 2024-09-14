#include "ros/ros.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <stdlib.h>


/* Here are the links length btw joints for R1 (5DOF).               /L5
 *           length Unit in meter [m]                       ___L3___/
 */                  //                                    /
#define R1_L1 .120   //                                   /L2
#define R1_L2 .150   //                                  / 
#define R1_L3 .110   //                                 | 
#define R1_L4 .000   //                                 |L1
#define R1_L5 .021   //                                 |

/* Here are the links length btw joints for R2 (4DOF).
 *           length Unit in meter [m]
 */
#define R2_L1 .224
#define R2_L2 .250
#define R2_L3 .230
#define R2_L4 .171

/* These are the distance btw both arms axis.
 *           Units in meter [m]                                                         |
 */               //                                                                    |
#define LR1 .000  //                                ^ Y_worldFrame                      |          ^ Z_worldFrame
#define WR1 .310  //                                |                                   |          |
#define HR1 .000  //                                |                                   |          |
                  //            o----> Y_R1         o----> X_worldFrame      <--->      |          o----> X_worldFrame    <--->
#define LR2 .000  //            |                                              |        |                                   |
#define WR2 .390  //            |                                              |        |                                   |
#define HR2 .335  //            v X_R1                                         |        |                                   |
                  //                                ^ Y_R2                     |WR2     |          ^ Z_R2                   |HR2
                  //                                |                          |        |          |                        |
                  //                                |                          |        |          |                        |
                  //                                o----> X_R2              <--->      |          o----> X_R2            <--->
                  //                                                                    | 
                  //            ^        LR1        ^                                   |
                  //            |-------------------|                                   |
                  //            v                   v                                   |
                  //                                                                    |

#define R1_ori 0
#define R2_ori 0

/* This is [dt]: the time btw each update of speed
 * and the number of msgs that should be sent to Arduino
 * in order to make both arms sync togethe in real time megurment.
 */
#define increment 30 // unit in milli second [ms].

#define R2_wrist_zero_angle 120.0 // The fourth joint in R2  is a Servo motor, and this is the the zero angle of joint in Servo angles.
/* Transmition Values*/
#define R1BaseTr 3.3
#define R1ShoulderTr 5.0
#define R1ElbowTr 3.5
#define R1WristTr 2.3

#define R2BaseTr 1.0
#define R2ShoulderTr 4.5
#define R2ElbowTr 4.5

/* R1 Joints angles
 *  Units in [rad]
 */
float R1_q1 = M_PI / 2;
float R1_q2 = M_PI / 2;
float R1_q3 = -M_PI / 2;
float R1_q4 = 0;
float R1_q5=0;

/* R2 Joints angles
 *  Units in [rad]
 */
float R2_q1 = M_PI / 2;
float R2_q2 = M_PI / 2;
float R2_q3 = -M_PI / 2;
float R2_q4 = 0;

/* R1 Joints angular speeds
 *     Units in [rad/s]
 */
float R1_q1_dot = 0;
float R1_q2_dot = 0;
float R1_q3_dot = 0;
float R1_q4_dot = 0;
float R1_q5_dot = 0;

/* R1 Joints angular speeds
 *     Units in [rad/s]
 */
float R2_q1_dot = 0;
float R2_q2_dot = 0;
float R2_q3_dot = 0;
float R2_q4_dot = 0;

/* Point coordinates in (World Frame)*/
float px = 0;
float py = 0;
float pz = 0;

/* Convert from point coordinates in (World Frame) to point coordinates in (R1 Frame)*/
float px_R1 = 0;
float py_R1 = 0;
float pz_R1 = 0;

/* Convert from point coordinates in (World Frame) to point coordinates in (R2 Frame)*/
float px_R2 = 0;
float py_R2 = 0;
float pz_R2 = 0;

float rad = .2;
float theta = 0;

/*Point Speed in (World Frame).
 *     Units in [m/s]
 */
float X_dot = 0.0;
float Y_dot = 0.0;
float Z_dot = 0.0; 

/*End effector Speed in (R1), which is in Jacobian Matrix called [P_dot].
 *                        Units in [m/s]
 */
float R1_X_dot = 0.0;
float R1_Y_dot = 0.0;
float R1_Z_dot = 0.0;
float R1_Wx = 0.0;
float R1_Wy = 0.0;
float R1_Wz = 0.0;

/*End effector Speed in (R2), which is in Jacobian Matrix called [P_dot].
 *                        Units in [m/s]
 */
float R2_X_dot = 0.0;
float R2_Y_dot = 0.0;
float R2_Z_dot = 0.0;
float R2_Wx = 0;
float R2_Wy = 0;
float R2_Wz = 0;

bool R1_gripper_state = false; // true: open the gripper, otherwise close it.
bool R2_gripper_state = false;

bool check = false; // Check state if bouth arms, moving or not.
/* The Mission that should be done.
   * It can holds one of these numbers:
   * mission = 0 : that means do the IK.
   * mission = 1 : that means do the FK.
   * mission = 2 : that means do the IJ.
   * mission = 3 : that means go to Zero.
   */
short int mission = 0;
  
/* this message is an array holds 12 values
 * it may contain angles of tow arm R1 & R2 in case of working in (IK,FK)
 * or angular speeds of Joints if we were working with Jacobian.
 */
std_msgs::Float32MultiArray Action_msg; 

/*Initializing the messege to be an array with size (1x11).
 * Elemnts of this Array orgenized as:
 * Elemnts(1,2,3,4,5): for R1 joints motors.
 * Elemnts(6,7,8,9): for R2 joints motors.
 * Elents (10): for R1 Gripper state.
 * Elents (11): for R2 Gripper state.
 */
void init_msg();

void fix_point_for_both_arms_frames(); // Convert from Point in WorldFrame to Point in each Arm Frame.

void inverse(); // Inverse Kinematic for both of Arms.
void forward(); // Forward Kinematic for both of Arms.

/* [Q_dot]: from Jacobian matrix, 
 * here there are equations to calculate
 * angular Speeds of Joints (both arms).
 * equations extracted from Jacobian Matrix
 * after I solved it in matlab. 
 */
void calculate_R2_q_dot();
void calculate_R1_q_dot();

void correct_R2_q3_dot(); // The third Joint angular Speed needs some modifing "special treatment" , beacuse of parallelogram Link.

void inv_jacobian(); // Inverse of Jacobian, in order to get Q_dot for each arm.

void update_angles(); // Update angles every [dt] seconds.

void rad_2_steps(); // after calculating IK or FK, it needs to be converted into Steps for Nema motors.
void radPerSecond_2_stepsPerSecond(); // after calculating angular Speeds, it needs to be converted into Steps/s for Nema motors.

void update_path_velocities();

void update_grippers_state(); // Open it or close it.

void checkCallback(const std_msgs::Bool::ConstPtr& msg){
    check = msg->data;
}
char** argv_global;
int main(int argc, char** argv) {
  argv_global = argv;
  
  ros::init(argc, argv, "DUAL");
  
  ros::NodeHandle n;
  
  ros::Publisher ACTINO_PUB = n.advertise<std_msgs::Float32MultiArray>("toggle_led", 1);
  
  ros::Subscriber CHECK = n.subscribe("check", 1, checkCallback);
  
  init_msg();
  
  ros::Rate loop_rate(increment);
  
  if (argc<2){
    ROS_INFO("\n\nError in ordered Mission :(\nYou have only 4 Missions:\n * mission = 0 : that means do the IK.\n * mission = 1 : that means do the FK.\n * mission = 2 : that means do the IJ.\n * mission = 3 : that means go to Zero.");
    return 1;
  }
  mission = atof(argv[1]);
  if (mission<0 ||mission>3){
    ROS_INFO("\n\nError in ordered Mission :(\nYou have only 4 Missions:\n * mission = 0 : that means do the IK.\n * mission = 1 : that means do the FK.\n * mission = 2 : that means do the IJ.\n * mission = 3 : that means go to Zero.");
    return 1;
  }
  /** these comming conditions to make shure that the user run the Node correctly **/
  if(mission==0 && argc!=5){
    ROS_INFO("\n\nthe correct input for inverse is:\nrosrun  dual_arms  Control_IK_FK_J  0  Px  Py  Pz");
    return 1;
  }
  else if(mission==0){
    px = atof(argv[2]);
    py = atof(argv[3]);
    pz = atof(argv[4]);
    ROS_INFO("\n\nyou entered Point:\nPx = %3.2f\nPy = %3.2f\nPz = %3.2f\n",px,py,pz);
  }
  if(mission==1 && argc!=11){
    ROS_INFO("\n\nthe correct input for Forward is:\nrosrun  dual_arms  Control_IK_FK_J  1  R1q1  R1q2  R1q3  R1q4  R1q5  R2q1  R2q2  R2q3  R2q4\n\t\tall angles in degree.");
    return 1;
  }
  else if(mission==1){
    ROS_INFO("\n\nyou entered angles:\nR1 angles:\ntheta1 = %3.2f\ntheta2 = %3.2f\ntheta3 = %3.2f\ntheta4 = %3.2f\ntheta5 = %3.2f\nR2 angle:\ntheta1 = %3.2f\ntheta2 = %3.2f\ntheta3 = %3.2f\ntheta4 = %3.2f\n",atof(argv_global[2]),atof(argv_global[3]),atof(argv_global[4]),atof(argv_global[5]),atof(argv_global[6]),atof(argv_global[7]),
atof(argv_global[8]),atof(argv_global[9]),atof(argv_global[10]));
  }  
  if (mission==2 && argc!=2){
    ROS_INFO("\n\nthe correct input for Jacobian is:\nrosrun  dual_arms  Control_IK_FK_J  2");
    return 1;
  }
  if (mission==3 && argc!=2){
    ROS_INFO("\n\nthe correct input for Zero Pose is:\nrosrun  dual_arms  Control_IK_FK_J  3");
    return 1;
  }
/**********************************/
  
  while (ros::ok()) {
     Action_msg.data.clear(); // clear data of last angles smg.
     Action_msg.data.push_back(mission);
     if (mission == 2) {
       inv_jacobian();
       radPerSecond_2_stepsPerSecond();
       update_path_velocities();
     }
    else if (mission == 0) {
      inverse();
      rad_2_steps();
    }
    else if (mission == 1){
      forward();
      rad_2_steps();
    }
    update_grippers_state();
    
    ACTINO_PUB.publish(Action_msg);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
void init_msg() {
  std_msgs::MultiArrayDimension msg_dim;
  msg_dim.label = "angles";
  msg_dim.size = 11; // size is [11] 'cause we have 11 angles of tow arms.
  msg_dim.stride = 11;
  Action_msg.layout.dim.clear(); // clean layout.
  Action_msg.layout.dim.push_back(msg_dim); // sets layout of angles msg.
}
void fix_point_for_both_arms_frames(){
  px_R1 = -py + LR1;
  py_R1 = px + WR1;
  pz_R1 = pz + HR1;

  px_R2 = px + LR2;
  py_R2 = py + WR2;
  pz_R2 = pz + HR2;
}
void inverse(){
  fix_point_for_both_arms_frames();
  
  float A, B, C, a, b, r;
  /**** R2 ****/
  R2_q1 = atan2(py_R2, px_R2); // theta 1
  if (!py_R2 && !px_R2)
    R2_q1 = 0;
    
  A = px_R2 - R2_L4 * cos(R2_q1) * cos(R2_ori);
  B = py_R2 - R2_L4 * sin(R2_q2) * cos(R2_ori);
  C = pz_R2 - R2_L1 - R2_L4 * sin(R2_ori);
  
  R2_q3 = -acos((A * A + B * B + C * C - R2_L2 * R2_L2 - R2_L3 * R2_L3) / (2 * R2_L2 * R2_L3)); // theta 3
  
  a = R2_L3 * sin(R2_q3);
  b = R2_L2 + R2_L3 * cos(R2_q3);
  r = sqrt(a * a + b * b);
  
  R2_q2 = atan2(C, sqrt(r * r - C * C)) - atan2(a, b); // theta 2
  
  R2_q4 = R2_ori - R2_q2 - R2_q3; // theta 4
                                     
  /****R1****/
  R1_q1 = atan2(py_R1, px_R1); // theta 1
  if (!py_R1 && !px_R1)
    R1_q1 = 0;
  
  A = px_R1 - R1_L4 * cos(R1_q1) * cos(R1_ori);
  B = py_R1 - R1_L4 * sin(R1_q2) * cos(R1_ori);
  C = pz_R1 - R1_L1 - R1_L4 * sin(R1_ori);
  
  R1_q3 = -acos((A * A + B * B + C * C - R1_L2 * R1_L2 - R1_L3 * R1_L3) / (2 * R1_L1 * R1_L3)); // theta 3
  
  a = R1_L3 * sin(R1_q3);
  b = R1_L2 + R1_L3 * cos(R1_q3);
  r = sqrt(a * a + b * b);
  
  R1_q2 = atan2(C, sqrt(r * r - C * C)) - atan2(a, b); // theta 2
  
  R1_q4 = R1_ori - R1_q2 - R1_q3; // theta 4
  
  R1_q3 = R1_q3 - atan((R1_L2 * cos(R1_q2)) / ( R1_L2 * sin(R1_q2)));
  R1_q2 = M_PI - R1_q2;
  R2_q2 = M_PI - R2_q2;
   R2_q3 = -M_PI/2.0 -(-R2_q3-2*135*M_PI/180.0) - R2_q2;
}
void forward(){
 R2_q3 = -90.0 -(-135-atof(argv_global[9])-135) - atof(argv_global[8]);
  
  R1_q1 = atof(argv_global[2])*M_PI/180.0;
  R1_q2 = atof(argv_global[3])*M_PI/180.0;
  R1_q3 = atof(argv_global[4])*M_PI/180.0;
  R1_q4 = atof(argv_global[5])*M_PI/180.0;
  R1_q5 = atof(argv_global[6])*M_PI/180.0;
  R2_q1 = atof(argv_global[7])*M_PI/180.0;
  R2_q2 = atof(argv_global[8])*M_PI/180.0;
  R2_q3 = R2_q3*M_PI/180.0;
  R2_q4 = atof(argv_global[10])*M_PI/180.0;
}
void calculate_R2_q_dot(){
  R2_q1_dot = -(R2_Y_dot*cos(R2_q1) - R2_X_dot*sin(R2_q1))/(R2_L3*sin(R2_q3) - R2_L2*cos(R2_q2) + R2_L4*sin(R2_q3 - R2_q4));


    R2_q2_dot = (R2_L3*R2_Z_dot*cos(R2_q3) - R2_L3*R2_L4*R2_q4_dot*sin(R2_q4) + R2_L4*R2_Z_dot*cos(R2_q3)*cos(R2_q4) + R2_L3*R2_X_dot*cos(R2_q1)*sin(R2_q3) + R2_L3*R2_Y_dot*sin(R2_q1)*sin(R2_q3) + R2_L4*R2_Z_dot*sin(R2_q3)*sin(R2_q4) - R2_L4*R2_X_dot*cos(R2_q1)*cos(R2_q3)*sin(R2_q4) + R2_L4*R2_X_dot*cos(R2_q1)*cos(R2_q4)*sin(R2_q3) - R2_L4*R2_Y_dot*cos(R2_q3)*sin(R2_q1)*sin(R2_q4) + R2_L4*R2_Y_dot*cos(R2_q4)*sin(R2_q1)*sin(R2_q3))/(R2_L2*R2_L4*cos(R2_q2 + R2_q3 - R2_q4) + R2_L2*R2_L3*cos(R2_q2 + R2_q3));


    R2_q3_dot = -(R2_Z_dot*sin(R2_q2) + R2_X_dot*cos(R2_q1)*cos(R2_q2) + R2_Y_dot*cos(R2_q2)*sin(R2_q1) - R2_L4*R2_q4_dot*cos(R2_q2)*cos(R2_q3 - R2_q4) + R2_L4*R2_q4_dot*sin(R2_q2)*sin(R2_q3 - R2_q4))/(R2_L3*cos(R2_q2 + R2_q3) + R2_L4*cos(R2_q2 + R2_q3 - R2_q4));


    R2_q4_dot = (2*R2_Z_dot*cos(R2_q1 + R2_q2) - 2*R2_Y_dot*cos(R2_q2) + R2_Y_dot*cos(2*R2_q1 + R2_q2) - 2*R2_Z_dot*cos(R2_q1 - R2_q2) - R2_X_dot*sin(2*R2_q1 + R2_q2) + R2_Y_dot*cos(2*R2_q1 - R2_q2) - R2_X_dot*sin(2*R2_q1 - R2_q2) + 4*R2_L4*R2_Wx*cos(R2_q2 + R2_q3 - R2_q4) + 4*R2_L3*R2_Wx*cos(R2_q2 + R2_q3))/(4*R2_L3*cos(R2_q2 + R2_q3)*sin(R2_q1));
}
void calculate_R1_q_dot(){
  R1_q1_dot=(R1_Y_dot*cos(R1_q1) - R1_X_dot*sin(R1_q1))/(R1_L3*cos(R1_q2 + R1_q3) + R1_L2*cos(R1_q2) + R1_L4*cos(R1_q2 + R1_q3 + R1_q4) - R1_L5*sin(R1_q2 + R1_q3 + R1_q4));
  
  R1_q2_dot=(R1_Z_dot*cos(R1_q2)*sin(R1_q3) + R1_Z_dot*cos(R1_q3)*sin(R1_q2) - R1_Y_dot*sin(R1_q1)*sin(R1_q2)*sin(R1_q3) - R1_L5*R1_Wy*cos(R1_q1)*cos(R1_q4) - R1_L4*R1_Wy*cos(R1_q1)*sin(R1_q4) + R1_L5*R1_Wx*cos(R1_q4)*sin(R1_q1) + R1_L4*R1_Wx*sin(R1_q1)*sin(R1_q4) + R1_X_dot*cos(R1_q1)*cos(R1_q2)*cos(R1_q3) + R1_Y_dot*cos(R1_q2)*cos(R1_q3)*sin(R1_q1) - R1_X_dot*cos(R1_q1)*sin(R1_q2)*sin(R1_q3))/(R1_L2*sin(R1_q3));
  
  R1_q3_dot=-(R1_L2*R1_Z_dot*sin(R1_q2) + R1_L2*R1_X_dot*cos(R1_q1)*cos(R1_q2) + R1_L2*R1_Y_dot*cos(R1_q2)*sin(R1_q1) + R1_L3*R1_Z_dot*cos(R1_q2)*sin(R1_q3) + R1_L3*R1_Z_dot*cos(R1_q3)*sin(R1_q2) + R1_L3*R1_X_dot*cos(R1_q1)*cos(R1_q2)*cos(R1_q3) + R1_L3*R1_Y_dot*cos(R1_q2)*cos(R1_q3)*sin(R1_q1) - R1_L3*R1_X_dot*cos(R1_q1)*sin(R1_q2)*sin(R1_q3) - R1_L3*R1_Y_dot*sin(R1_q1)*sin(R1_q2)*sin(R1_q3) - R1_L3*R1_L5*R1_Wy*cos(R1_q1)*cos(R1_q4) - R1_L3*R1_L4*R1_Wy*cos(R1_q1)*sin(R1_q4) + R1_L3*R1_L5*R1_Wx*cos(R1_q4)*sin(R1_q1) + R1_L3*R1_L4*R1_Wx*sin(R1_q1)*sin(R1_q4) - R1_L2*R1_L5*R1_Wy*cos(R1_q1)*cos(R1_q3)*cos(R1_q4) - R1_L2*R1_L4*R1_Wy*cos(R1_q1)*cos(R1_q3)*sin(R1_q4) - R1_L2*R1_L4*R1_Wy*cos(R1_q1)*cos(R1_q4)*sin(R1_q3) + R1_L2*R1_L5*R1_Wx*cos(R1_q3)*cos(R1_q4)*sin(R1_q1) + R1_L2*R1_L4*R1_Wx*cos(R1_q3)*sin(R1_q1)*sin(R1_q4) + R1_L2*R1_L4*R1_Wx*cos(R1_q4)*sin(R1_q1)*sin(R1_q3) + R1_L2*R1_L5*R1_Wy*cos(R1_q1)*sin(R1_q3)*sin(R1_q4) - R1_L2*R1_L5*R1_Wx*sin(R1_q1)*sin(R1_q3)*sin(R1_q4))/(R1_L2*R1_L3*sin(R1_q3));
  
  R1_q4_dot=(R1_Z_dot*sin(R1_q2) + R1_X_dot*cos(R1_q1)*cos(R1_q2) + R1_Y_dot*cos(R1_q2)*sin(R1_q1) - R1_L3*R1_Wy*cos(R1_q1)*sin(R1_q3) + R1_L3*R1_Wx*sin(R1_q1)*sin(R1_q3) - R1_L5*R1_Wy*cos(R1_q1)*cos(R1_q3)*cos(R1_q4) - R1_L4*R1_Wy*cos(R1_q1)*cos(R1_q3)*sin(R1_q4) - R1_L4*R1_Wy*cos(R1_q1)*cos(R1_q4)*sin(R1_q3) + R1_L5*R1_Wx*cos(R1_q3)*cos(R1_q4)*sin(R1_q1) + R1_L4*R1_Wx*cos(R1_q3)*sin(R1_q1)*sin(R1_q4) + R1_L4*R1_Wx*cos(R1_q4)*sin(R1_q1)*sin(R1_q3) + R1_L5*R1_Wy*cos(R1_q1)*sin(R1_q3)*sin(R1_q4) - R1_L5*R1_Wx*sin(R1_q1)*sin(R1_q3)*sin(R1_q4))/(R1_L3*sin(R1_q3));
  
  R1_q5_dot=-(R1_Wx*cos(R1_q1) + R1_Wy*sin(R1_q1))/sin(R1_q2 + R1_q3 + R1_q4);
}
void inv_jacobian(){
  calculate_R2_q_dot();
  calculate_R1_q_dot();
  
  update_angles();
}
void update_angles() {
  R1_q1 = R1_q1 + R1_q1_dot / increment;
  R1_q2 = R1_q2 + R1_q2_dot / increment;
  R1_q3 = R1_q3 + R1_q3_dot / increment;
  R1_q4 = R1_q4 + R1_q4_dot / increment;
  
  R2_q1 = R2_q1 + R2_q1_dot / increment;
  R2_q2 = R2_q2 + R2_q2_dot / increment;
  R2_q3 = R2_q3 + R2_q3_dot / increment;
  R2_q4 = R2_q4 + R2_q4_dot / increment;
}
void rad_2_steps(){
  //R1
  Action_msg.data.push_back(R1_q1 * 6400.0 * R1BaseTr / (2 * M_PI));
  Action_msg.data.push_back(R1_q2 * 6400.0 * R1ShoulderTr / (2 * M_PI));
  Action_msg.data.push_back(R1_q3 * 6400.0 * R1ElbowTr / (2 * M_PI));
  Action_msg.data.push_back(R1_q4 * 6400.0 * R1WristTr/ (2 * M_PI));
  Action_msg.data.push_back(R1_q5 * 6400.0 / (2 * M_PI));
  //R2
  Action_msg.data.push_back(R2_q1 * 6400.0 * R2BaseTr / (2 * M_PI));
  Action_msg.data.push_back(R2_q2 * 6400.0 * R2ShoulderTr / (2 * M_PI));
  Action_msg.data.push_back(R2_q3 * 6400.0 * R2ElbowTr / (2 * M_PI));
  Action_msg.data.push_back(int(R2_wrist_zero_angle - R2_q4 * 180.0 / M_PI));
}
void radPerSecond_2_stepsPerSecond(){
  //R1
  Action_msg.data.push_back(R1_q1_dot * 6400.0 * R1BaseTr / (2 * M_PI));
  Action_msg.data.push_back(R1_q2_dot * 6400.0 * R1ShoulderTr / (2 * M_PI));
  Action_msg.data.push_back(R1_q3_dot * 6400.0 * R1ElbowTr / (2 * M_PI));
  Action_msg.data.push_back(R1_q4_dot * 6400.0 * R1WristTr / (2 * M_PI));
  Action_msg.data.push_back(R1_q5_dot * 6400.0 / (2 * M_PI));
  //R2
  Action_msg.data.push_back(R2_q1_dot * 6400.0 * R2BaseTr / (2 * M_PI));
  Action_msg.data.push_back(-R2_q2_dot * 6400.0 * R2ShoulderTr / (2 * M_PI));
  Action_msg.data.push_back(-R2_q3_dot * 6400.0 * R2ElbowTr / (2 * M_PI));
  Action_msg.data.push_back(int(R2_wrist_zero_angle - R2_q4 * 180.0 / M_PI));
}
void correct_R2_q3_dot() {
  //R2_q3_dot = R2_q3_dot + R2_q2_dot;
}
void update_path_velocities() {
  R2_X_dot = rad * cos(theta);
  R2_Y_dot = rad * sin(theta);
  R2_Z_dot = .008;
    
  theta += M_PI / increment;
}
void update_grippers_state() {
  R1_gripper_state = 0;
  R2_gripper_state = 0;
  Action_msg.data.push_back(R1_gripper_state);
  Action_msg.data.push_back(R2_gripper_state);
}
