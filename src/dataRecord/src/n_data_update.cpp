
#include <ros/ros.h>
#include <std_msgs/Time.h>

// standard
#include <sstream>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cstdlib>

#define RESET "\033[0m"
#define PURPLE "\033[35m"
namespace fs = boost::filesystem;

// 获取文件夹下的所有文件名
std::vector<std::string> getFilesInFolder(const std::string &folder)
{
    std::vector<std::string> files;
    DIR *dir = opendir(folder.c_str());

    if (dir == nullptr)
    {
        std::cout << "Invalid directory path or directory does not exist." << std::endl;
        return files;
    }

    if (dir)
    {
        struct dirent *ent;
        while ((ent = readdir(dir)) != nullptr)
        {
            if (ent->d_type == DT_REG)
            {
                files.push_back(ent->d_name);
            }
        }
        closedir(dir);
    }
    return files;
}

// 获取文件大小
uint64_t getFileSize(const std::string &file_path)
{
    struct stat st;
    if (stat(file_path.c_str(), &st) == 0)
    {
        return st.st_size;
    }
    return 0;
}

void checkFolderSize(const ros::TimerEvent &, std::string bag_path, int max_folder_size)
{
    std::cout << "检查数据大小" << std::endl;
    std::vector<std::string> files = getFilesInFolder(bag_path);
    uint64_t total_size = 0;

    for (const auto &file : files)
    {
        std::string file_path = bag_path + "/" + file;
        total_size += getFileSize(file_path);
    }

    int totalMB = static_cast<int>(total_size / 1024);

    std::cout << "FolderSize: " << totalMB << " MB" << std::endl;

    // 如果总大小超出限定，则依次删除时间最前的文件
    while (totalMB > max_folder_size && !files.empty())
    {
        auto pFiles = std::min_element(files.begin(), files.end()); // 复杂度O(n），但最坏需要循环n次
        // 判断该文件是不是已经录制完成的bag文件，如果是正在录制的，则不删除
        std::string filename = *pFiles;
        size_t dotPosition = filename.find_last_of(".");
        std::string fileExtension = filename.substr(dotPosition + 1);
        if (fileExtension == "bag")
        {
            std::string oldest_file = bag_path + "/" + *pFiles;
            uint64_t file_size = getFileSize(oldest_file);
            std::cout << "delete file:" << oldest_file << ",size:" << file_size / 1024 << " MB" << std::endl;
            remove(oldest_file.c_str());
            totalMB -= static_cast<int>(file_size / 1024);
        }
        files.erase(pFiles);
    }
}

void backupCallback(const std_msgs::Time::ConstPtr &msg, const std::string &dataFolderPath, const std::string &backupFolderPath, std::set<std::string> &fileForBackup)
{
    std::cout << "收到异常时间戳，开始备份" << std::endl;
    // backup the file which was recording in last round
    for (auto file : fileForBackup)
    {
        if (boost::filesystem::exists(dataFolderPath + "/" + file)) // Have not been recorded
        {
            continue; // bag are still record
        }
        size_t dotPosition = file.find_last_of(".");
        std::string filename = file.substr(0, dotPosition - 1);         // Extract the  part "yyyy-MM-dd-HH-mm-ss_x.bag"
        if (boost::filesystem::exists(dataFolderPath + "/" + filename)) // Have been recorded
        {
            if (!boost::filesystem::exists(backupFolderPath + "/" + filename)) // Have been backuped
            {
                fs::copy_file(dataFolderPath + "/" + filename, backupFolderPath + "/" + filename, fs::copy_option::overwrite_if_exists);
            }
            fileForBackup.erase(file);
        }
    }

    // Convert ROS Time to boost::posix_time::ptime
    boost::posix_time::ptime givenTime = boost::posix_time::from_time_t(msg->data.sec);

    // Create a vector to store file paths in the data folder
    std::vector<fs::path> filePaths;
    // Iterate through the data folder and store file paths
    for (const auto &entry : boost::make_iterator_range(fs::directory_iterator(dataFolderPath), {}))
    {
        if (fs::is_regular_file(entry.path()))
        {
            std::cout << "backup: " << entry << std::endl;
            filePaths.push_back(entry.path());
        }
    }
    // Sort the file paths based on their names (timestamps)
    std::sort(filePaths.begin(), filePaths.end());

    fs::path backupFile;

    // Find the backup file based on the given time
    for (size_t i = 0; i < filePaths.size() - 1; ++i)
    {
        std::cout<<i<<std::endl;
        std::string fileName1 = filePaths[i].filename().string();
        std::string fileName2 = filePaths[i + 1].filename().string();
        fileName1 = fileName1.substr(1, 19).replace(10, 1, " ").replace(13, 1, ":").replace(16, 1, ":"); //  Convert "yyyy-MM-dd-HH-mm-ss" to "YYYY-MM-DD HH:mm:ss"
        fileName2 = fileName1.substr(1, 19).replace(10, 1, " ").replace(13, 1, ":").replace(16, 1, ":");
        boost::posix_time::ptime timestamp1 = boost::posix_time::time_from_string(fileName1); // Extract the timestamp part "yyyy-MM-dd-HH-mm-ss"
        boost::posix_time::ptime timestamp2 = boost::posix_time::time_from_string(fileName2);



        if (timestamp1 <= givenTime && givenTime < timestamp2)
        {
            backupFile = filePaths[i];
            break;
        }
    }

    if (!backupFile.empty())
    {
        // Copy the backup file to the backup folder
        if (!boost::filesystem::exists(backupFolderPath / backupFile.filename()))
        {
            fs::copy_file(backupFile, backupFolderPath / backupFile.filename(), fs::copy_option::overwrite_if_exists);
        }
    }
    else if (!filePaths.empty())
    {
        // If no backup file found, but there are files, then the last file has the highest timestamp
        // Check if the last file's timestamp is still smaller than the given time
        boost::posix_time::ptime maxTimestamp = boost::posix_time::time_from_string(filePaths.back().filename().string());
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
                    fs::copy_file(backupFile, backupFolderPath / backupFile.filename(), fs::copy_option::overwrite_if_exists);
                }
            }
            else if (fileExtension == "active") // If the file is recording, wait for the next round
            {
                fileForBackup.insert(filePaths.back().filename().string());
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "n_data_update");
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

    // 创建定时器定时检查文件大小
    ros::Timer timerCheckFolderSize = nh.createTimer(ros::Duration(check_interval), boost::bind(&checkFolderSize, _1, bag_path, max_folder_size));

    ros::spin();
    return 0;
}