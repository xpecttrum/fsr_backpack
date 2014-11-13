
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>

#include <sensor_msgs/Imu.h>

#include <uwsim_bullet/LinearMath/btQuaternion.h>
#include <uwsim_bullet/LinearMath/btMatrix3x3.h>


#include <tf_conversions/tf_eigen.h>

ros::Subscriber imu_sub;

class FSRBackpackArm
{
    public:
        FSRBackpackArm(ros::NodeHandle &nh);

        void init(ros::NodeHandle &pnh);
        void spinOnce();

        void jointStateSpinner();
        void stop_Motor();

    private:
        ros::Publisher  m_joint_state_pub_;
        ros::Publisher  m_joint_pub_;
        ros::Subscriber m_joint_sub_;

        void setGoal(const trajectory_msgs::JointTrajectory::ConstPtr& msg);

        void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);

        double linear_position;
        double rotation_position;

        bool checkJointNames(const std::vector<std::string> *joint_names);
        //bool checkGoal(const std::vector<trajectory_msgs::JointTrajectoryPoint> *points);

        void write(double lift, double lift_speed, double sweep, double sweep_speed, double sweep_acceleration);

        std::string lift_joint_;
        std::string sweep_joint_;

        ros::Time start_time_;
        ros::Time goal_start_time_;
        std::list<trajectory_msgs::JointTrajectoryPoint> trajectory_;
        std::vector<control_msgs::JointTolerance> path_tolerance_;
        std::vector<control_msgs::JointTolerance> goal_tolerance_;
        ros::Duration goal_time_tolerance_;

        ros::Time last_update_;
        double last_lift_;
        double last_sweep_;

        double lift_;
        double sweep_;
        double lift_speed_;
        double sweep_speed_;
        double min_lift_;
        double max_lift_;
        double max_lift_speed_;
        double min_sweep_;
        double max_sweep_;
        double max_sweep_speed_;
        double lift_tolerance_;
        double sweep_tolerance_;
        ros::Duration time_tolerance_;

        int lift_index_;
        int sweep_index_;

        int max_sweep_steps_;

        double joint_state_rate_;

        bool got_new_goal_;

//        double acceleration_coeficient_;
        double speed_coeficient_;
};

FSRBackpackArm::FSRBackpackArm(ros::NodeHandle &nh){ //Construtor
    // Publishers : Only publish the most recent reading
    m_joint_pub_ = nh.advertise<control_msgs::JointTrajectoryControllerState>("/arm_controller/state", 1);

    // Subscribers : Only subscribe to the most recent instructions
    m_joint_sub_ = nh.subscribe("/arm_controller/command", 1, &FSRBackpackArm::setGoal, this);

    //m_joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 200);
    //<remap from="joint_states" to="/arm_controller/joint_states"/>
    m_joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/arm_controller/joint_states", 200);

    imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 10, &FSRBackpackArm::imuCallback,this);

   
}

void FSRBackpackArm::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    //geometry_msgs::Quaternion orientation;

    //Convert quaternion to RPY.
    tf::Quaternion q;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(imu_msg->orientation, q);
    tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
    ROS_DEBUG("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);

   rotation_position = yaw;
   
   //fprintf(stderr,"\n new %lf",rotation_position);
   linear_position = 0.0;



}

void FSRBackpackArm::init(ros::NodeHandle &pnh){
    sweep_joint_ = "arm_axel_joint";

    linear_position = 0.0;
    rotation_position = 0.0;

 start_time_ = ros::Time::now();

    pnh.param("min_lift", min_lift_, -0.25);
    pnh.param("max_lift", max_lift_, 0.25);
    pnh.param("max_lift_speed", max_lift_speed_, 1.0);

    pnh.param("min_sweep", min_sweep_, -1.5);
    pnh.param("max_sweep", max_sweep_, 1.5);
    pnh.param("max_sweep_speed", max_sweep_speed_, 1.0);
	max_sweep_steps_ = 3;

    pnh.param("lift_tolerance", lift_tolerance_, 0.05);
    pnh.param("sweep_tolerance", sweep_tolerance_, 0.05);

    double time_tolerance;
    pnh.param("time_tolerance", time_tolerance, 0.5);
    time_tolerance_ = ros::Duration(time_tolerance);

    pnh.param("joint_state_rate", joint_state_rate_, 50.0);
	 
}
       

bool FSRBackpackArm::checkJointNames(const std::vector<std::string> *joint_names)
{
    int found_joint = 0;
    for(int i=0 ; i<joint_names->size() ; i++)
    {
        if(lift_joint_.compare(joint_names->at(i)) == 0)
        {
            lift_index_ = i;
            found_joint++;
        }
        else if(sweep_joint_.compare(joint_names->at(i)) == 0)
        {
            sweep_index_ = i;
            found_joint++;
        }
    }
    return (joint_names->size() == found_joint);
}



void FSRBackpackArm::jointStateSpinner()
{
    //fprintf(stderr,"\njointStateSpinner");

    ros::Rate r(joint_state_rate_);
    while(ros::ok())
    {
        //aqui tenho que enviar somente as juntas que existem, senao nao funciona - fazer rostopic echo /joint_states para verificar
        sensor_msgs::JointState msg;

      /*  msg.header.stamp = ros::Time::now();
        msg.name.push_back(lift_joint_);
        msg.position.push_back(lift_);
        msg.velocity.push_back(lift_speed_);
        msg.effort.push_back(0.0); */
        
        msg.name.push_back(sweep_joint_);
        msg.position.push_back(sweep_);
        msg.velocity.push_back(sweep_speed_);
        msg.effort.push_back(0.0);
        
        /*msg.name.push_back("lower_arm_joint");
        msg.position.push_back(lift_);
        msg.velocity.push_back(lift_speed_);
        msg.effort.push_back(0.0);
        
        msg.name.push_back("metal_detector_arm_joint");
        msg.position.push_back(-1.0*lift_);
        msg.velocity.push_back(-1.0*lift_speed_);
        msg.effort.push_back(0.0);
*/

        //fprintf(stderr,"\nm_joint_state_pub_");
        m_joint_state_pub_.publish(msg);
        //fprintf(stderr," ok");

        r.sleep();
        //fprintf(stderr,"\nm_joint_state_pub_ - fim");

    }
}

void FSRBackpackArm::spinOnce()
{
    //get positions
    //posicoes sao preenchidas no callback do imu
    // linear_position
    // rotation_position

   
    //fprintf(stderr,"\nspinOnce");

    ros::Duration delta_t = ros::Time::now() - last_update_;
    lift_ = -1.0*(linear_position*(max_lift_ - min_lift_)/4048 + min_lift_);
    lift_speed_ = (lift_ - last_lift_)/delta_t.toSec();
    
    delta_t = ros::Time::now() - last_update_;
    //sweep_ = rotation_position*(max_sweep_ - min_sweep_)/max_sweep_steps_ + min_sweep_;
    sweep_speed_ = (sweep_ - last_sweep_)/delta_t.toSec();

	sweep_ = -rotation_position;
    //fprintf(stderr,"\n sweep %lf",sweep_);
  
    // Publish Position & Speed
    control_msgs::JointTrajectoryControllerState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.joint_names.push_back(lift_joint_);
    joint_state.actual.positions.push_back(lift_);
    joint_state.actual.velocities.push_back(lift_speed_);
    joint_state.joint_names.push_back(sweep_joint_);
    joint_state.actual.positions.push_back(sweep_);
    joint_state.actual.velocities.push_back(sweep_speed_);

    //fprintf(stderr,"\nPublish Position & Speed ok");

    if(trajectory_.size() > 0)
    {
        joint_state.desired.positions.push_back(trajectory_.front().positions[lift_index_]);
        joint_state.desired.velocities.push_back(trajectory_.front().velocities[lift_index_]);
        joint_state.error.positions.push_back(trajectory_.front().positions[lift_index_] - lift_);
        joint_state.error.velocities.push_back(trajectory_.front().velocities[lift_index_] - lift_speed_);
        joint_state.desired.positions.push_back(trajectory_.front().positions[sweep_index_]);
        joint_state.desired.velocities.push_back(trajectory_.front().velocities[sweep_index_]);
        joint_state.error.positions.push_back(trajectory_.front().positions[sweep_index_] - sweep_);
        joint_state.error.velocities.push_back(trajectory_.front().velocities[sweep_index_] - sweep_speed_);
        joint_state.desired.time_from_start = trajectory_.front().time_from_start;
        joint_state.actual.time_from_start = ros::Time::now() - start_time_;
        joint_state.error.time_from_start = ros::Time::now() - start_time_ - trajectory_.front().time_from_start;
    }

    //fprintf(stderr,"\nVou publicar o joint_state");

    m_joint_pub_.publish(joint_state); //isto é que faz a TF mexer
    //fprintf(stderr,"\nOK");

    last_update_ = ros::Time::now();
    last_lift_ = lift_;
    last_sweep_ = sweep_;

    //fprintf(stderr,"\nfim");
    
}

void FSRBackpackArm::setGoal(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
  
}


void FSRBackpackArm::stop_Motor()
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fsr_backpack_arm");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    ROS_INFO("FSR Backpack Arm - collect arm position and change TFs");

    FSRBackpackArm arm(n);
    arm.init(pn);

    fprintf(stderr,"init ok");
	ros::AsyncSpinner spinner(5);
    spinner.start();
    
    boost::thread joint_state_thread(&FSRBackpackArm::jointStateSpinner, &arm);

    ros::Rate r(10.0);
    while(ros::ok())
    {
        arm.spinOnce();
        //fprintf(stderr,".");
        r.sleep();
        //fprintf(stderr,"-");

    }

    joint_state_thread.join();

    fprintf(stderr,"\n Spinner stop");
    spinner.stop();

    return 0;
}
