#include <ros/ros.h>
#include <std_msgs/Time.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "exceptional_topic_publisher");
    ros::NodeHandle nh;

    // Create a publisher for the /exceptionalTopic topic
    ros::Publisher pub = nh.advertise<std_msgs::Time>("/exceptionalTopic", 1);

    ros::Rate loop_rate(1); // 1 Hz

    while (ros::ok())
    {
        std_msgs::Time msg;
        msg.data = ros::Time::now(); // Set the message data to the current time

        // Publish the message on the /exceptionalTopic topic
        pub.publish(msg);

        // Process any pending ROS callbacks
        ros::spinOnce();

        // Sleep to maintain the desired publishing rate
        loop_rate.sleep();
    }

    return 0;
}
