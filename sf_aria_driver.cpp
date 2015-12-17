#ifdef ADEPT_PKG
  #include <Aria.h>
#else
  #include <Aria/Aria.h>
#endif

#include "ros/ros.h"
#include "std_msgs/ByteMultiArray.h" //for sending and receiving 40 chars of midi#include "std_msgs/ByteMultiArray.h" //for sending and receiving 40 chars of midi
#include "RosAria.hpp"

class AriaMidi //midi driver for SF
{
public:
    AriaMidi(ArRobot *robot, ros::NodeHandle nh) : robot_(robot), nh_(nh)
    {

    }
    ~AriaMidi() {}

    void init();
    void receiveCB(const std_msgs::ByteMultiArray::ConstPtr& msg);
    void playTones(char * tones, int size);

protected:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ArRobot *robot_;
};

void AriaMidi::init()
{
    subscriber_ = nh_.subscribe("aria_midi", 10, &AriaMidi::receiveCB, this);
}

void AriaMidi::receiveCB(const std_msgs::ByteMultiArray::ConstPtr& msg)
{
    char tones[40];
    int i =0 ;

    ROS_INFO("I heard mode incoming");

    for(std::vector<int8_t>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
    {
        tones[i] = *it;
        i++;
    }
    playTones(tones, i);

}

void AriaMidi::playTones(char * tones, int size)
{
    printf("Data decoded:\n");
    for(int i=0; i<size; i++)
        printf("%d ", tones[i]);
    printf("\n");
    robot_->lock();
    robot_->comStrN(ArCommands::SAY,tones, size);
    robot_->unlock();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "RosAria");
    ros::NodeHandle nh(std::string("~"));
	Aria::init();
	
    RosAriaNode *node = new RosAriaNode(nh);

	ArRobot *robot_;
	robot_ = new ArRobot;
	
	if( node->Setup(robot_) !=0)  //modify this so it take the pointer can be used outside the class
	{
	    ROS_FATAL( "RosAria: ROS node setup failed... \n" );
	    return -1;
	}


    AriaMidi *ariaMidi = new AriaMidi( robot_, nh);
    ariaMidi->init(); //init tune node

    ros::Rate loop_rate(1000);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete node;

    ROS_INFO( "sf_AriaDriver: Quitting... \n" );
    return 0;
}
