// ros
#include <ros/ros.h>

#define RESET "\033[0m"
#define PURPLE "\033[35m"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "n_data_record");
    ros::NodeHandle nh;

    // 读取参数
    std::string bag_path;
    std::string backup_path;
    int data_partition;
    int bag_size;
    int bag_duration;
    std::vector<std::string> topic_names;
    int check_interval;
    int max_folder_size;

    nh.param<std::string>("/n_data_record/bag_path", bag_path, "./");
    nh.param<std::string>("/n_data_record/backup_path", backup_path, "./");
    nh.param<int>("/n_data_record/data_partition", data_partition, 1);
    nh.param<int>("/n_data_record/bag_size", bag_size, 1024);
    nh.param<int>("/n_data_record/bag_duration", bag_duration, 3600);
    nh.param<std::vector<std::string>>("/n_data_record/topic_names", topic_names, std::vector<std::string>());
    nh.param<int>("/n_data_record/check_interval", check_interval, 3600);
    nh.param<int>("/n_data_record/max_folder_size", max_folder_size, 3600);

    // 输出读取到的参数值
    std::cout << PURPLE << std::string(20, '-') << "parameters of n_data_record:" << std::string(20, '-') << RESET << std::endl;
    ROS_INFO_STREAM(PURPLE << "bag_path: " << RESET << bag_path);
    ROS_INFO_STREAM(PURPLE << "backup_path: " << RESET << backup_path);
    ROS_INFO_STREAM(PURPLE << "data_partition: " << RESET << data_partition);
    ROS_INFO_STREAM(PURPLE << "bag_size: " << RESET << bag_size);
    ROS_INFO_STREAM(PURPLE << "bag_duration: " << RESET << bag_duration << "(Byte)");
    for (const auto &topic : topic_names)
    {
        ROS_INFO_STREAM(PURPLE << "topic: " << RESET << topic);
    }
    ROS_INFO_STREAM(PURPLE << "check_interval: " << RESET << check_interval);
    ROS_INFO_STREAM(PURPLE << "max_folder_size: " << RESET << max_folder_size);
    std::cout << PURPLE << std::string(68, '-') << RESET << std::endl;

    // 录制数据
    if (data_partition == 1)
    {
        std::string command = "rosbag record -o " + bag_path + "/" + " --split --duration=" + std::to_string(bag_duration); // 要执行的终端指令
        for (const auto &topic : topic_names)
        {
            command = command + " " + topic;
        }
        int result = std::system(command.c_str()); // 这条指令运行后，主线程会堵塞，直到按下ctrl+c
    }
    else if (data_partition == 2)
    {
        std::string command = "rosbag record -o " + bag_path + "/" + " --split --size=" + std::to_string(bag_size) ; // 要执行的终端指令
        for (const auto &topic : topic_names)
        {
            command = command + " " + topic;
        }
        int result = std::system(command.c_str());
    }

    return 0;
}
