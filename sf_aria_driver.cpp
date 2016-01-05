#ifdef ADEPT_PKG
  #include <Aria.h>
#else
  #include <Aria/Aria.h>
#endif

#include "ros/ros.h"
#include "std_msgs/ByteMultiArray.h" //for sending and receiving 40 chars of midi#include "std_msgs/ByteMultiArray.h" //for sending and receiving 40 chars of midi
#include "std_msgs/UInt8.h"
#include "RosAria.hpp"

#include "sf_dance/DanceSequence.h"
#include "sf_dance/DanceStep.h"

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

class AriaDance
{
public:
    AriaDance(ArRobot *robot, ros::NodeHandle nh) : robot_(robot), nh_(nh)
    {

    }

    ~AriaDance() {}

    void init()
    {
        subscriber_ = nh_.subscribe("aria_dance_sequence", 10, &AriaDance::receiveCB, this);
    }

    void receiveCB(const sf_dance::DanceSequence::ConstPtr& msg)
    {
        if (nextDanceSequence_ == true)
        {
            std::vector<sf_dance::DanceStep> danceStep;

            ROS_INFO("I hear rotAccel : %f",  msg->rotAccel);

            danceSequence_.rotAccel = msg->rotAccel;
            danceSequence_.rotDecel =  msg->rotDecel;
            danceSequence_.transAccel = msg->transAccel;
            danceSequence_.transDecel = msg->transDecel;

            danceSequence_.danceStep.clear();
            for(std::vector<sf_dance::DanceStep>::const_iterator it = msg->danceStep.begin(); it != msg->danceStep.end(); ++it)
            {
               danceSequence_.danceStep.push_back(*it);
            }

            prepareDance();

        }
    }

    void prepareDance()
    {
        nextDanceSequence_ = false;
        nextDanceStep_ = true;
        DelayTime_ = 0;
        StartTimer_ = 0;

        it_ = danceSequence_.danceStep.begin();

    }

    void performDance()
    {
        static unsigned char mode = 2;

        if (nextDanceStep_ == true) //perform the dance step
        {
            if (it_ != danceSequence_.danceStep.end()) //still got dance step left
            {
                mode = it_->mode;
                switch (mode)
                {
                    case sf_dance::DanceSequence::ROTATION :
                        DelayTime_ = 800;
                        robot_->lock();
                        robot_->setDeltaHeading(it_->value);
                        robot_->unlock();
                        StartTimer_ = ArUtil::getTime();
                        ROS_INFO("Perform dance");
                    break;

                    case sf_dance::DanceSequence::TRANSLATION :
                        DelayTime_ = 800;
                        robot_->lock();
                        robot_->move(it_->value);
                        robot_->unlock();
                        StartTimer_ = ArUtil::getTime();
                        ROS_INFO("Perform dance");
                    break;

                }
                nextDanceStep_ = 0;
                ++it_; //next dance step
            }
            else
            {
                nextDanceSequence_ = true; //ready to receive next dance sequence

            }
        }

        if ( (ArUtil::getTime() - StartTimer_) >= DelayTime_ )
        {
            switch (mode)
            {
                case sf_dance::DanceSequence::ROTATION :
                    robot_->lock();
                    if (robot_->isHeadingDone(1))
                        nextDanceStep_ = true;
                    robot_->unlock();
                break;

                case sf_dance::DanceSequence::TRANSLATION :
                    robot_->lock();
                    if (robot_->isHeadingDone(1))
                        nextDanceStep_ = true;
                    robot_->unlock();
                break;

            }


        }
    }

protected:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ArRobot *robot_;
    sf_dance::DanceSequence danceSequence_;
    unsigned int DelayTime_;
    unsigned int StartTimer_;
    std::vector<sf_dance::DanceStep>::const_iterator it_;
    bool nextDanceSequence_;
    bool nextDanceStep_;


};

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

    AriaDance *ariaDance = new AriaDance( robot_, nh);
    ariaDance->init();

    ros::Rate loop_rate(1000);

    while (ros::ok())
    {
        ariaDance->performDance();
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete node;

    ROS_INFO( "sf_AriaDriver: Quitting... \n" );
    return 0;
}
