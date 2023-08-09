
#include <ros/ros.h>
#include <std_msgs/Time.h>

// standard
#include <sstream>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <vector>
#include <algorithm>
#include <iostream>
#include <cstdlib>

#define RESET "\033[0m"
#define PURPLE "\033[35m"
namespace fs = boost::filesystem;

void backupCallback(const std_msgs::Time::ConstPtr &msg, const std::string &dataFolderPath, const std::string &backupFolderPath, std::set<std::string> &fileForBackup)
{
    std::cout <<PURPLE<<"Received abnormal data timestamp, starting backup."<<RESET<< std::endl;

    std::set<std::string> filesSucceedBackup;

    // backup the file which was recording in last round
    for (auto file : fileForBackup)
    {
        size_t dotPosition = file.find_last_of(".");
        std::string filename = file.substr(0, dotPosition); // Extract the  part "yyyy-MM-dd-HH-mm-ss_x.bag"
        if (boost::filesystem::exists(dataFolderPath + "/" + file)) // Have not been recorded
        {
            continue; // bag are still record
        }
        if (boost::filesystem::exists(dataFolderPath + "/" + filename)) // Have been recorded
        {
            if (!boost::filesystem::exists(backupFolderPath + "/" + filename)) // Have been backuped
            {
                fs::copy_file(dataFolderPath + "/" + filename, backupFolderPath + "/" + filename, fs::copy_option::none);
                std::cout << "backup file: " << backupFolderPath + "/" + filename << std::endl;
            }
            filesSucceedBackup.insert(file);
        }
    }
    for (const auto &file : filesSucceedBackup)
    {
        fileForBackup.erase(file);
    }
    // Convert ROS Time to boost::posix_time::ptime
    boost::posix_time::ptime givenTime = msg->data.toBoost(); // utc time
    givenTime = givenTime + boost::posix_time::hours(8);      // cn time

    // Create a vector to store file paths in the data folder
    std::vector<fs::path> filePaths;
    // Iterate through the data folder and store file paths
    for (const auto &entry : boost::make_iterator_range(fs::directory_iterator(dataFolderPath), {}))
    {
        if (fs::is_regular_file(entry.path()))
        {
            filePaths.push_back(entry.path());
        }
    }

    if (filePaths.size() == 1)
    {
        fs::path backupFile;
        std::string fileName = filePaths[0].filename().string();
        fileName = fileName.substr(1, 19);                                               // Extract "yyyy-MM-dd-HH-mm-ss"
        fileName = fileName.replace(10, 1, " ").replace(13, 1, ":").replace(16, 1, ":"); //  Convert "yyyy-MM-dd-HH-mm-ss" to "YYYY-MM-DD HH:mm:ss"
        // Check if the only file's timestamp is smaller than the given time
        boost::posix_time::ptime timestamp = boost::posix_time::time_from_string(fileName);
        if (timestamp < givenTime)
        {
            // Add the last file to the returnFileNames vector
            size_t dotPosition = filePaths.back().filename().string().find_last_of(".");
            std::string fileExtension = filePaths.back().filename().string().substr(dotPosition + 1);
            if (fileExtension == "bag")
            {
                backupFile = filePaths.back().filename().string();
                if (!boost::filesystem::exists(backupFolderPath / backupFile.filename()))
                {
                    std::cout << "backup file: " << backupFile.filename() << std::endl;
                    fs::copy_file(backupFile, backupFolderPath / backupFile.filename(), fs::copy_option::overwrite_if_exists);
                }
            }
            else if (fileExtension == "active") // If the file is recording, wait for the next round
            {
                std::cout << "bag is recording, Delayed backup " << std::endl;
                fileForBackup.insert(filePaths.back().filename().string());
            }
        }
    }
    else if (filePaths.size() > 1)
    {
        // Sort the file paths based on their names (timestamps)
        std::sort(filePaths.begin(), filePaths.end());

        fs::path backupFile;

        // Find the backup file based on the given time
        for (size_t i = 0; i < filePaths.size() - 1; ++i)
        {
            std::string fileName1 = filePaths[i].filename().string();
            std::string fileName2 = filePaths[i + 1].filename().string();
            fileName1 = fileName1.substr(1, 19);                                               // Extract "yyyy-MM-dd-HH-mm-ss"
            fileName1 = fileName1.replace(10, 1, " ").replace(13, 1, ":").replace(16, 1, ":"); //  Convert "yyyy-MM-dd-HH-mm-ss" to "YYYY-MM-DD HH:mm:ss"
            fileName2 = fileName2.substr(1, 19);
            fileName2 = fileName2.replace(10, 1, " ").replace(13, 1, ":").replace(16, 1, ":");
            boost::posix_time::ptime timestamp1 = boost::posix_time::time_from_string(fileName1);
            boost::posix_time::ptime timestamp2 = boost::posix_time::time_from_string(fileName2);
            if (timestamp1 <= givenTime && givenTime < timestamp2)
            {
                backupFile = filePaths[i];
                break;
            }
        }

        if (!backupFile.empty())
        {
            size_t dotPosition = backupFile.filename().string().find_last_of(".");
            std::string fileExtension = filePaths.back().filename().string().substr(dotPosition + 1);

            if (fileExtension == "bag")
            {
                backupFile = filePaths.back().filename().string();
                if (!boost::filesystem::exists(backupFolderPath / backupFile.filename()))
                {
                    std::cout << "backup file: " << backupFile.filename() << std::endl;
                    fs::copy_file(backupFile, backupFolderPath / backupFile.filename(), fs::copy_option::overwrite_if_exists);
                }
            }
            else if (fileExtension == "active") // If the file is recording, wait for the next round
            {
                fileForBackup.insert(filePaths.back().filename().string());
            }
        }
        else // check if the bag need to backup is the last file
        {
            std::cout << "check if the bag need to backup is the last file" << std::endl;
            std::string lastFileName = filePaths.back().filename().string();
            lastFileName = lastFileName.substr(1, 19);                                               // Extract "yyyy-MM-dd-HH-mm-ss"
            lastFileName = lastFileName.replace(10, 1, " ").replace(13, 1, ":").replace(16, 1, ":"); //  Convert "yyyy-MM-dd-HH-mm-ss" to "YYYY-MM-DD HH:mm:ss"
            // Check if the only file's timestamp is smaller than the given time
            boost::posix_time::ptime maxTimestamp = boost::posix_time::time_from_string(lastFileName);
            if (maxTimestamp < givenTime)
            {
                // Add the last file to the returnFileNames vector
                size_t dotPosition = filePaths.back().filename().string().find_last_of(".");
                std::string fileExtension = filePaths.back().filename().string().substr(dotPosition + 1);
                if (fileExtension == "bag")
                {
                    backupFile = filePaths.back().filename().string();
                    if (!boost::filesystem::exists(backupFolderPath / backupFile.filename()))
                    {
                        std::cout << "backup file: " << backupFile.filename() << std::endl;
                        fs::copy_file(backupFile, backupFolderPath / backupFile.filename(), fs::copy_option::overwrite_if_exists);
                    }
                }
                else if (fileExtension == "active") // If the file is recording, wait for the next round
                {
                    std::cout << "bag is recording, Delayed backup " << std::endl;
                    fileForBackup.insert(filePaths.back().filename().string());
                }
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "n_data_backup");
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
    std::cout << PURPLE << std::string(20, '-') << "parameters of n_data_update:" << std::string(20, '-') << RESET << std::endl;
    ROS_INFO_STREAM(PURPLE << "bag_path: " << RESET << bag_path);
    ROS_INFO_STREAM(PURPLE << "backup_path: " << RESET << backup_path);
    ROS_INFO_STREAM(PURPLE << "data_partition: " << RESET << data_partition);
    ROS_INFO_STREAM(PURPLE << "bag_size: " << RESET << bag_size << "(Byte)");
    ROS_INFO_STREAM(PURPLE << "bag_duration: " << RESET << bag_duration << " s");
    for (const auto &topic : topic_names)
    {
        ROS_INFO_STREAM(PURPLE << "topic: " << RESET << topic);
    }
    ROS_INFO_STREAM(PURPLE << "check_interval: " << RESET << check_interval);
    ROS_INFO_STREAM(PURPLE << "max_folder_size: " << RESET << max_folder_size);
    std::cout << PURPLE << std::string(68, '-') << RESET << std::endl;

    std::set<std::string> fileForBackup;
    // 订阅异常话题，从数据录取文件夹中备份对应数据至异常数据文件夹
    ros::Subscriber sub = nh.subscribe<std_msgs::Time>("/exceptionalTopic", 10, boost::bind(&backupCallback, _1, bag_path, backup_path, boost::ref(fileForBackup)));

    ros::spin();
    return 0;
}