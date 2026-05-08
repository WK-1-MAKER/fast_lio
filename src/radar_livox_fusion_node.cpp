#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

namespace
{
using CloudXYZI = pcl::PointCloud<pcl::PointXYZI>;

class RadarLivoxFusionNode
{
public:
    RadarLivoxFusionNode(ros::NodeHandle &nh)
        : nh_(nh)
    {
        nh_.param<std::string>("input/livox_topic", livox_topic_, std::string("/livox/lidar"));
        nh_.param<std::string>("input/mmwave_topic", mmwave_topic_, std::string("/mmwave/points"));
        nh_.param<std::string>("output/fusion_topic", fusion_topic_, std::string("/fusion/points"));
        nh_.param<std::string>("output/livox_raw_topic", livox_raw_topic_, std::string("/livox/lidar_pcl2_raw"));
        nh_.param<std::string>("output/frame_id", output_frame_id_, std::string("livox"));

        nh_.param<int>("livox/scan_line", livox_scan_line_, 6);
        nh_.param<int>("livox/point_filter_num", livox_point_filter_num_, 2);
        nh_.param<double>("common/max_sync_dt", max_sync_dt_, 0.1);
        nh_.param<bool>("filter/is_filter", is_filter_, true);
        nh_.param<double>("filter/min_range", min_range_, 1.0);
        nh_.param<double>("filter/max_range", max_range_, 120.0);
        nh_.param<double>("filter/voxel_leaf", voxel_leaf_, 0.05);

        std::vector<double> extrinsic_t;
        std::vector<double> extrinsic_r;
        nh_.param<std::vector<double>>("extrinsic/mmwave_to_livox_t", extrinsic_t, std::vector<double>{0.0, 0.0, 0.0});
        nh_.param<std::vector<double>>("extrinsic/mmwave_to_livox_r", extrinsic_r, std::vector<double>{1.0, 0.0, 0.0,
                                                                                                       0.0, 1.0, 0.0,
                                                                                                       0.0, 0.0, 1.0});
        if (extrinsic_t.size() != 3 || extrinsic_r.size() != 9)
        {
            ROS_WARN("Invalid extrinsic size, fallback to identity.");
            extrinsic_t = {0.0, 0.0, 0.0};
            extrinsic_r = {1.0, 0.0, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 1.0};
        }
        t_m2l_ = Eigen::Vector3d(extrinsic_t[0], extrinsic_t[1], extrinsic_t[2]);
        r_m2l_ << extrinsic_r[0], extrinsic_r[1], extrinsic_r[2],
                  extrinsic_r[3], extrinsic_r[4], extrinsic_r[5],
                  extrinsic_r[6], extrinsic_r[7], extrinsic_r[8];

        sub_livox_ = nh_.subscribe(livox_topic_, 10, &RadarLivoxFusionNode::livoxCallback, this);
        sub_mmwave_ = nh_.subscribe(mmwave_topic_, 50, &RadarLivoxFusionNode::mmwaveCallback, this);
        pub_fusion_ = nh_.advertise<sensor_msgs::PointCloud2>(fusion_topic_, 10);
        pub_livox_raw_ = nh_.advertise<sensor_msgs::PointCloud2>(livox_raw_topic_, 10);

        voxel_filter_.setLeafSize(static_cast<float>(voxel_leaf_), static_cast<float>(voxel_leaf_), static_cast<float>(voxel_leaf_));

        ROS_INFO_STREAM("radar_livox_fusion_node started.\n"
                        << "  livox_topic: " << livox_topic_ << "\n"
                        << "  mmwave_topic: " << mmwave_topic_ << "\n"
                        << "  livox_raw_topic: " << livox_raw_topic_ << "\n"
                        << "  fusion_topic: " << fusion_topic_ << "\n"
                        << "  output_frame: " << output_frame_id_);
    }

private:
    void mmwaveCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_mmwave_ = msg;
    }

    void livoxCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg)
    {
        CloudXYZI::Ptr livox_cloud(new CloudXYZI());
        CloudXYZI::Ptr livox_raw_cloud(new CloudXYZI());
        CloudXYZI::Ptr mmwave_cloud(new CloudXYZI());
        CloudXYZI::Ptr fused_cloud(new CloudXYZI());

        buildLivoxRawCloud(msg, livox_raw_cloud);
        publishCloud(livox_raw_cloud, msg->header.stamp, output_frame_id_, pub_livox_raw_);

        buildLivoxCloud(msg, livox_cloud);
        if (livox_cloud->empty())
        {
            return;
        }

        sensor_msgs::PointCloud::ConstPtr mmwave_msg;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            mmwave_msg = latest_mmwave_;
        }

        if (mmwave_msg)
        {
            double dt = std::fabs((msg->header.stamp - mmwave_msg->header.stamp).toSec());
            bool use_mmwave = is_filter_ ? (dt <= max_sync_dt_) : true;
            if (use_mmwave)
            {
                buildMmwaveCloud(mmwave_msg, mmwave_cloud);
                transformMmwaveToLivox(mmwave_cloud);
            }
        }

        *fused_cloud = *livox_cloud;
        if (!mmwave_cloud->empty())
        {
            fused_cloud->insert(fused_cloud->end(), mmwave_cloud->begin(), mmwave_cloud->end());
        }

        CloudXYZI::Ptr output_cloud(new CloudXYZI());
        if (is_filter_) 
        {
            voxel_filter_.setInputCloud(fused_cloud);
            voxel_filter_.filter(*output_cloud);
        }
        else
        {
            *output_cloud = *fused_cloud;
        }

        publishCloud(output_cloud, msg->header.stamp, output_frame_id_, pub_fusion_);
    }

    void buildLivoxRawCloud(const livox_ros_driver::CustomMsg::ConstPtr &msg, CloudXYZI::Ptr &out)
    {
        out->clear();
        out->reserve(msg->point_num);

        for (uint32_t i = 0; i < msg->point_num; ++i)
        {
            const auto &p = msg->points[i];
            pcl::PointXYZI q;
            q.x = p.x;
            q.y = p.y;
            q.z = p.z;
            q.intensity = static_cast<float>(p.reflectivity);
            out->push_back(q);
        }
    }

    void buildLivoxCloud(const livox_ros_driver::CustomMsg::ConstPtr &msg, CloudXYZI::Ptr &out)
    {
        out->clear();
        out->reserve(msg->point_num);

        int valid_count = 0;
        for (uint32_t i = 1; i < msg->point_num; ++i)
        {
            const auto &p = msg->points[i];
            if (p.line >= livox_scan_line_)
            {
                continue;
            }
            if (!(((p.tag & 0x30) == 0x10) || ((p.tag & 0x30) == 0x00)))
            {
                continue;
            }

            ++valid_count;
            if (valid_count % std::max(1, livox_point_filter_num_) != 0)
            {
                continue;
            }

            pcl::PointXYZI q;
            q.x = p.x;
            q.y = p.y;
            q.z = p.z;
            q.intensity = static_cast<float>(p.reflectivity);
            if (is_filter_ && !rangePass(q))
            {
                continue;
            }
            out->push_back(q);
        }
    }

    void buildMmwaveCloud(const sensor_msgs::PointCloud::ConstPtr &msg, CloudXYZI::Ptr &out)
    {
        out->clear();
        out->reserve(msg->points.size());

        int intensity_channel = -1;
        for (size_t channel_id = 0; channel_id < msg->channels.size(); ++channel_id)
        {
            if (msg->channels[channel_id].name == "intensity")
            {
                intensity_channel = static_cast<int>(channel_id);
                break;
            }
        }

        for (size_t i = 0; i < msg->points.size(); ++i)
        {
            pcl::PointXYZI q;
            q.x = msg->points[i].x;
            q.y = msg->points[i].y;
            q.z = msg->points[i].z;
            q.intensity = 1.0f;

            if (intensity_channel >= 0 && i < msg->channels[intensity_channel].values.size())
            {
                q.intensity = msg->channels[intensity_channel].values[i];
            }

            if (is_filter_ && !rangePass(q))
            {
                continue;
            }
            out->push_back(q);
        }
    }

    void transformMmwaveToLivox(CloudXYZI::Ptr &cloud)
    {
        for (auto &point : cloud->points)
        {
            Eigen::Vector3d p_m(point.x, point.y, point.z);
            Eigen::Vector3d p_l = r_m2l_ * p_m + t_m2l_;
            point.x = static_cast<float>(p_l.x());
            point.y = static_cast<float>(p_l.y());
            point.z = static_cast<float>(p_l.z());
        }
    }

    bool rangePass(const pcl::PointXYZI &point) const
    {
        double r2 = static_cast<double>(point.x) * point.x + static_cast<double>(point.y) * point.y + static_cast<double>(point.z) * point.z;
        return r2 >= min_range_ * min_range_ && r2 <= max_range_ * max_range_;
    }

    void publishCloud(const CloudXYZI::Ptr &cloud,
                      const ros::Time &stamp,
                      const std::string &frame_id,
                      const ros::Publisher &publisher)
    {
        sensor_msgs::PointCloud2 out_msg;
        pcl::toROSMsg(*cloud, out_msg);
        out_msg.header.stamp = stamp;
        out_msg.header.frame_id = frame_id;
        publisher.publish(out_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_livox_;
    ros::Subscriber sub_mmwave_;
    ros::Publisher pub_fusion_;
    ros::Publisher pub_livox_raw_;

    std::string livox_topic_;
    std::string mmwave_topic_;
    std::string fusion_topic_;
    std::string livox_raw_topic_;
    std::string output_frame_id_;

    int livox_scan_line_ = 6;
    int livox_point_filter_num_ = 2;
    double max_sync_dt_ = 0.03;
    bool is_filter_ = true;
    double min_range_ = 1.0;
    double max_range_ = 120.0;
    double voxel_leaf_ = 0.2;

    Eigen::Matrix3d r_m2l_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_m2l_ = Eigen::Vector3d::Zero();

    std::mutex mutex_;
    sensor_msgs::PointCloud::ConstPtr latest_mmwave_;

    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;
};
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_livox_fusion_node");
    ros::NodeHandle nh("~");

    RadarLivoxFusionNode node(nh);
    ros::spin();
    return 0;
}
