#include "cloud_convert.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
// #include <execution>

namespace zjloc
{

    /*void CloudConvert::Process(const livox_ros_driver::CustomMsg::ConstPtr &msg, std::vector<point3D> &pcl_out)
    {
        AviaHandler(msg);
        pcl_out = cloud_out_;
    }*/

    void CloudConvert::Process(const sensor_msgs::msg::PointCloud2::ConstPtr &msg,
                               std::vector<point3D> &pcl_out)
    {
        switch (lidar_type_)
        {
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;

        case LidarType::ROBOSENSE16:
            RobosenseHandler(msg);
            break;

        case LidarType::PANDAR:
            PandarHandler(msg);
            break;
        case LidarType::DTOF:
            DTofHander(msg);
            break;
        case LidarType::MTOF:
            MTofHander(msg);
            break;
        default:
            LOG(ERROR) << "Error LiDAR Type: " << int(lidar_type_);
            break;
        }
        pcl_out = cloud_out_;
    }

    /*
    void CloudConvert::AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();
        int plsize = msg->point_num;
        cloud_out_.reserve(plsize);

        static double tm_scale = 1e9;

        double headertime = msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9;
        timespan_ = msg->points.back().offset_time / tm_scale;

        // std::cout << "span:" << timespan_ << ",0: " << msg->points[0].offset_time / tm_scale
        //           << " , 100: " << msg->points[100].offset_time / tm_scale << std::endl;

        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(msg->points[i].x) &&
                  std::isfinite(msg->points[i].y) &&
                  std::isfinite(msg->points[i].z)))
                continue;

            if (i % point_filter_num_ != 0)
                continue;

            double range = msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y +
                           msg->points[i].z * msg->points[i].z;
            if (range > 150 * 150 || range < blind * blind)
                continue;
            if(std::tan(msg->points[i].x/msg->points[i].y)>1.732 || std::tan(msg->points[i].x/msg->points[i].y)<-1.732)
            // if(msg->points[i].y<=0 || std::tan(msg->points[i].x/msg->points[i].y)>1.732 || std::tan(msg->points[i].x/msg->points[i].y)<-1.732)
            {
                continue;
            }
            if(std::tan(msg->points[i].z/msg->points[i].y)>0.44 || std::tan(msg->points[i].z/msg->points[i].y)<-0.44)
            {
                continue;
            }

            if (/*(msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
            {
                point3D point_temp;
                point_temp.raw_point = Eigen::Vector3d(msg->points[i].x, msg->points[i].y, msg->points[i].z);
                point_temp.point = point_temp.raw_point;
                point_temp.relative_time = msg->points[i].offset_time / tm_scale; // curvature unit: ms
                point_temp.intensity = msg->points[i].reflectivity;

                point_temp.timestamp = headertime + point_temp.relative_time;
                point_temp.alpha_time = point_temp.relative_time / timespan_;
                point_temp.timespan = timespan_;
                point_temp.ring = msg->points[i].line;

                cloud_out_.push_back(point_temp);
            }
        }
    }
    */

    void CloudConvert::Oust64Handler(const sensor_msgs::msg::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();
        pcl::PointCloud<ouster_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();
        cloud_out_.reserve(plsize);

        static double tm_scale = 1e9;

        double headertime = msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9;
        timespan_ = pl_orig.points.back().t / tm_scale;
        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].t / tm_scale
        //           << " , 100: " << pl_orig.points[100].t / tm_scale
        //           << std::endl;

        for (int i = 0; i < pl_orig.points.size(); i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % point_filter_num_ != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].t / tm_scale; // curvature unit: ms
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;

            cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert::RobosenseHandler(const sensor_msgs::msg::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();
        pcl::PointCloud<robosense_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();
        cloud_out_.reserve(plsize);

        double headertime = msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9;
        //  FIXME:  时间戳大于0.1
        auto time_list_robosense = [&](robosense_ros::Point &point_1, robosense_ros::Point &point_2)
        {
            return (point_1.timestamp < point_2.timestamp);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_robosense);
        while (pl_orig.points[plsize - 1].timestamp - pl_orig.points[0].timestamp >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }

        timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;

        // std::cout << timespan_ << std::endl;

        // std::cout << pl_orig.points[1].timestamp - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9 - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9 - pl_orig.points.back().timestamp << std::endl;

        for (int i = 0; i < pl_orig.points.size(); i++)
        {
            // if (i % point_filter_num_ != 0)
            //     continue;
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].timestamp - pl_orig.points[0].timestamp; // curvature unit: s
            point_temp.intensity = pl_orig.points[i].intensity;

            // point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.timestamp = pl_orig.points[i].timestamp;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;
            if (point_temp.alpha_time > 1 || point_temp.alpha_time < 0)
                std::cout << point_temp.alpha_time << ", this may error." << std::endl;

            cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert::VelodyneHandler(const sensor_msgs::msg::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();

        pcl::PointCloud<velodyne_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        cloud_out_.reserve(plsize);

        double headertime = msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9;

        static double tm_scale = 1; //   1e6 - nclt kaist or 1

        //  FIXME:  nclt 及kaist时间戳大于0.1
        auto time_list_velodyne = [&](velodyne_ros::Point &point_1, velodyne_ros::Point &point_2)
        {
            return (point_1.time < point_2.time);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_velodyne);
        while (pl_orig.points[plsize - 1].time / tm_scale >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }
        timespan_ = pl_orig.points.back().time / tm_scale;
        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].time / tm_scale << " , 100: " << pl_orig.points[100].time / tm_scale << std::endl;

        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % point_filter_num_ != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].time / tm_scale; // curvature unit: s
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;

            cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert::PandarHandler(const sensor_msgs::msg::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();

        pcl::PointCloud<pandar_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        cloud_out_.reserve(plsize);

        double headertime = msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9;

        static double tm_scale = 1; //   1e6

        auto time_list_pandar = [&](pandar_ros::Point &point_1, pandar_ros::Point &point_2)
        {
            return (point_1.timestamp < point_2.timestamp);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_pandar);
        while (pl_orig.points[plsize - 1].timestamp - pl_orig.points[0].timestamp >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }
        timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;

        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[1].timestamp - pl_orig.points[0].timestamp
        //           << " , 100: " << pl_orig.points[100].timestamp - pl_orig.points[0].timestamp
        //           << msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9 - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9 - pl_orig.points.back().timestamp << std::endl;

        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % point_filter_num_ != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].timestamp - pl_orig.points[0].timestamp;
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;

            cloud_out_.push_back(point_temp);
        }
    }

    /**
     * sensor_msgs::msg::PointCloud2 存在timestamp的属性，并且时间单位是s
    */
    void CloudConvert::DTofHander(const sensor_msgs::msg::PointCloud2::ConstPtr &msg){

        cloud_out_.clear();
        cloud_full_.clear();

        pcl::PointCloud<dtof_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();
        cloud_out_.reserve(plsize);

        // 点云的时间戳
        double headertime = msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9;
        //std::cout << ANSI_COLOR_GREEN << "header time ..."<<headertime << ANSI_COLOR_RESET << std::endl;
        //  FIXME:  时间戳大于0.1
        auto time_list_dtof = [&](dtof_ros::Point &point_1, dtof_ros::Point &point_2){
            return (point_1.timestamp < point_2.timestamp);
        };
        // 将每个点按照时间戳进行排序
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_dtof);
        // 剔除timespan > 0.1的点 // timestamp 
        while (pl_orig.points[plsize - 1].timestamp - pl_orig.points[0].timestamp >= 0.1){
            plsize--;
            pl_orig.points.pop_back();
        }

        // 计算timespan_
        timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;
        // std::cout << ANSI_COLOR_GREEN << "  time span  ..."<<std::setprecision(13)<<timespan_ << ANSI_COLOR_RESET << std::endl;
        // std::cout << ANSI_COLOR_GREEN << "  time begin: "<<std::setprecision(13)<<pl_orig.points[0].timestamp
        //                               << "  time end: "<<pl_orig.points.back().timestamp<< ANSI_COLOR_RESET <<std::endl;

        for (int i = 0; i < pl_orig.points.size(); i++){
           
            // if (i % point_filter_num_ != 0)
            //     continue;
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;

            // 仅保留[0.1m, 15m]范围内的点
            if (range > 15 * 15 || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].timestamp - pl_orig.points[0].timestamp; // curvature unit: s
            point_temp.intensity = pl_orig.points[i].intensity;

            // point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.timestamp = pl_orig.points[i].timestamp;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_; // 这个点云的时间跨度
            point_temp.ring = pl_orig.points[i].ring;
            if (point_temp.alpha_time > 1 || point_temp.alpha_time < 0)
                std::cout << point_temp.alpha_time << ", this may error." << std::endl;

            cloud_out_.push_back(point_temp);
        }

        // std::cout << ANSI_COLOR_GREEN << " alpha time begin: "<<std::setprecision(13)<<cloud_out_[0].alpha_time
        //                               << " alpha time end: "<<cloud_out_.back().alpha_time<< ANSI_COLOR_RESET <<std::endl;

    }

    /**
     * sensor_msgs::msg::PointCloud2 存在timestamp的属性，并且时间单位是s
    */
    void CloudConvert::MTofHander(const sensor_msgs::msg::PointCloud2::ConstPtr &msg){

        cloud_out_.clear();
        cloud_full_.clear();

        pcl::PointCloud<dtof_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();
        cloud_out_.reserve(plsize);

        // 点云的时间戳
        double headertime = msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9;
        //std::cout << ANSI_COLOR_GREEN << "header time ..."<<headertime << ANSI_COLOR_RESET << std::endl;
    

        // 计算timespan_
        timespan_ = 0;
        // std::cout << ANSI_COLOR_GREEN << "  time span  ..."<<std::setprecision(13)<<timespan_ << ANSI_COLOR_RESET << std::endl;
        // std::cout << ANSI_COLOR_GREEN << "  time begin: "<<std::setprecision(13)<<pl_orig.points[0].timestamp
        //                               << "  time end: "<<pl_orig.points.back().timestamp<< ANSI_COLOR_RESET <<std::endl;

        for (int i = 0; i < pl_orig.points.size(); i++){
           
            // if (i % point_filter_num_ != 0)
            //     continue;
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;

            // 仅保留[0.1m, 15m]范围内的点
            if (range > 15 * 15 || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = 0; // curvature unit: s
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.alpha_time = 0;
            point_temp.timespan = timespan_; // 这个点云的时间跨度

            cloud_out_.push_back(point_temp);
        }

        // std::cout << ANSI_COLOR_GREEN << " alpha time begin: "<<std::setprecision(13)<<cloud_out_[0].alpha_time
        //                               << " alpha time end: "<<cloud_out_.back().alpha_time<< ANSI_COLOR_RESET <<std::endl;

    }

    void CloudConvert::LoadFromYAML(const std::string &yaml_file)
    {
        auto yaml = YAML::LoadFile(yaml_file);
        int lidar_type = yaml["preprocess"]["lidar_type"].as<int>();

        point_filter_num_ = yaml["preprocess"]["point_filter_num"].as<int>();
        blind = yaml["preprocess"]["blind"].as<double>();

        if (lidar_type == 1)
        {
            lidar_type_ = LidarType::AVIA;
            LOG(INFO) << "Using AVIA Lidar";
        }
        else if (lidar_type == 2)
        {
            lidar_type_ = LidarType::VELO32;
            LOG(INFO) << "Using Velodyne 32 Lidar";
        }
        else if (lidar_type == 3)
        {
            lidar_type_ = LidarType::OUST64;
            LOG(INFO) << "Using OUST 64 Lidar";
        }
        else if (lidar_type == 4)
        {
            lidar_type_ = LidarType::ROBOSENSE16;
            LOG(INFO) << "Using Robosense 16 LIdar";
        }
        else if (lidar_type == 5)
        {
            lidar_type_ = LidarType::PANDAR;
            LOG(INFO) << "Using Pandar LIdar";
        }
        else if (lidar_type == 6)
        {
            lidar_type_ = LidarType::DTOF;
            LOG(INFO) << "Using DTof Lidar (sensor_msgs/PointCloud2)";
        }
        else if (lidar_type == 7)
        {
            lidar_type_ = LidarType::MTOF;
            LOG(INFO) << "Using DTof Lidar (sensor_msgs/PointCloud2)";
        }
        else
        {
            LOG(WARNING) << "unknown lidar_type";
        }
    }

} // namespace zjloc