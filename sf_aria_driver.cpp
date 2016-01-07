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
        if (DoneDanceSequence_ == true) //ready for receiving next dance sequence
        {
            std::vector<sf_dance::DanceStep> danceStep;

            ROS_INFO("I hear rotAccel : %f",  msg->rotAccel);
            //may not need these variables
            danceSequence_.rotAccel = msg->rotAccel;
            danceSequence_.rotDecel =  msg->rotDecel;
            danceSequence_.transAccel = msg->transAccel;
            danceSequence_.transDecel = msg->transDecel;

            danceSequence_.danceStep.clear();   //clear the containers

            for(std::vector<sf_dance::DanceStep>::const_iterator it = msg->danceStep.begin(); it != msg->danceStep.end(); ++it)
            {
               danceSequence_.danceStep.push_back(*it);
            }

            prepareDance();

        }

    }

    void prepareDance()
    {
        DoneDanceSequence_ = false; // dance sequence to be performed
        NewDanceStep_ = true;   // new dance step to be performed

        it_ = danceSequence_.danceStep.begin();

        //can be used to setup the max speed etc.
        /*robot_->lock();

        if ( (danceSequence_.rotAccel != sf_dance::DanceSequence::DEFAULT)
             || (danceSequence_.rotDecel != sf_dance::DanceSequence::DEFAULT) )
        {
            robot_->setRotAccel(danceSequence_.rotAccel);
            robot_->setRotDecel(danceSequence_.rotDecel);
        }


        if ( (danceSequence_.transAccel != sf_dance::DanceSequence::DEFAULT)
             || (danceSequence_.transDecel != sf_dance::DanceSequence::DEFAULT) )
        {
            robot_->setTransAccel(danceSequence_.transAccel);
            robot_->setTransDecel(danceSequence_.transDecel);
        }

        robot_->unlock();
        */

    }

    void performDanceStep()
    {
        static unsigned char danceMode = 2; //0: rotation, 1: translation

        if (NewDanceStep_ == true) //ready to perform new dance step
        {
            if (it_ != danceSequence_.danceStep.end()) //still got dance step to be performed
            {
                danceMode = it_->mode;
                switch (danceMode)
                {
                    case sf_dance::DanceSequence::ROTATION :
                        robot_->lock();
                        robot_->setDeltaHeading(it_->value);
                        robot_->unlock();
                        ROS_INFO("Perform rot step");
                    break;

                    case sf_dance::DanceSequence::TRANSLATION :
                        robot_->lock();
                        robot_->move(it_->value);
                        robot_->unlock();
                        ROS_INFO("Perform trans step");
                    break;
                }
                NewDanceStep_ = false; //wait for the dance step to complete
                ++it_; //get next dance step
            }
            else    //no more dance step left, reset variables and wait for new danceSequence
            {
                DoneDanceSequence_ = true; //ready to receive next dance sequence
                NewDanceStep_ = false;  //no more new dance step
            }
        }

        if (DoneDanceSequence_== false) // if dance sequence not yet completed
        {
            switch (danceMode)
            {
                case sf_dance::DanceSequence::ROTATION :
                    robot_->lock();
                    if (robot_->isHeadingDone(1))
                        NewDanceStep_ = true; //ready for next dance step
                    robot_->unlock();
                break;

                case sf_dance::DanceSequence::TRANSLATION :
                    robot_->lock();
                    if (robot_->isMoveDone(1))
                        NewDanceStep_ = true; //ready for next dance step
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
    std::vector<sf_dance::DanceStep>::const_iterator it_;
    bool DoneDanceSequence_;
    bool NewDanceStep_;


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
        ariaDance->performDanceStep();
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete node;

    ROS_INFO( "sf_AriaDriver: Quitting... \n" );
    return 0;
}
