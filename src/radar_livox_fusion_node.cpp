#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <tuple>
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

struct EIGEN_ALIGN16 PointXYZIRT
{
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, time, time)
                                  (std::uint16_t, ring, ring))

namespace
{
using CloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using CloudXYZIRT = pcl::PointCloud<PointXYZIRT>;

enum class EnvironmentMode
{
    NORMAL,
    SMOKE_SUSPECTED,
    SMOKE
};

struct SmokeAdaptiveConfig
{
    bool enabled = false;
    int enter_frames = 5;
    int exit_frames = 15;
    double enter_score = 0.65;
    double exit_score = 0.35;
    double near_range = 8.0;
    double far_range = 20.0;
    double weight_valid_count = 0.35;
    double weight_far_low_reflectivity = 0.35;
    double weight_far_isolation = 0.30;
    double far_low_reflectivity = 8.0;
    double min_valid_points_ratio = 0.5;
    double isolated_voxel_leaf = 0.5;
    int isolated_voxel_max_points = 2;
    double radar_min_power_normal = 7.0;
    double radar_min_power_smoke = 5.5;
    int radar_max_points_normal = 300;
    int radar_max_points_smoke = 1000;
};

struct FrameQuality
{
    size_t valid_livox_points = 0;
    size_t far_livox_points = 0;
    size_t far_low_reflectivity_points = 0;
    size_t far_voxels = 0;
    size_t far_isolated_voxels = 0;
    size_t radar_points_before = 0;
    size_t radar_points_after = 0;
    size_t livox_points_after = 0;
    double baseline_valid_livox_points = 0.0;
    double valid_points_ratio = 1.0;
    double far_low_reflectivity_ratio = 0.0;
    double far_isolated_voxel_ratio = 0.0;
    double low_valid_point_score = 0.0;
    double far_low_reflectivity_score = 0.0;
    double far_isolated_voxel_score = 0.0;
    double smoke_score = 0.0;
    EnvironmentMode mode = EnvironmentMode::NORMAL;
};

struct LivoxPointCandidate
{
    PointXYZIRT point;
    double range = 0.0;
    float reflectivity = 0.0f;
    int far_voxel_count = 0;
};

struct RadarPointCandidate
{
    PointXYZIRT point;
    double range = 0.0;
    float power = 1.0f;
};

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
        nh_.param<double>("livox/scan_duration", fallback_livox_scan_duration_, 0.1);
        nh_.param<double>("sync/max_sync_dt", max_sync_dt_, max_sync_dt_);
        nh_.param<int>("sync/radar_queue_size", radar_queue_size_, 50);
        nh_.param<double>("sync/stale_time", radar_stale_time_, 0.2);
        int radar_ring = 0;
        nh_.param<int>("radar/ring", radar_ring, 0);
        nh_.param<bool>("filter/is_filter", is_filter_, true);
        nh_.param<double>("filter/min_range", min_range_, 1.0);
        nh_.param<double>("filter/max_range", max_range_, 120.0);
        nh_.param<double>("filter/voxel_leaf", voxel_leaf_, 0.05);
        nh_.param<bool>("smoke_adaptive/enabled", smoke_cfg_.enabled, false);
        nh_.param<int>("smoke_adaptive/enter_frames", smoke_cfg_.enter_frames, smoke_cfg_.enter_frames);
        nh_.param<int>("smoke_adaptive/exit_frames", smoke_cfg_.exit_frames, smoke_cfg_.exit_frames);
        nh_.param<double>("smoke_adaptive/enter_score", smoke_cfg_.enter_score, smoke_cfg_.enter_score);
        nh_.param<double>("smoke_adaptive/exit_score", smoke_cfg_.exit_score, smoke_cfg_.exit_score);
        nh_.param<double>("smoke_adaptive/range/near", smoke_cfg_.near_range, smoke_cfg_.near_range);
        nh_.param<double>("smoke_adaptive/range/far", smoke_cfg_.far_range, smoke_cfg_.far_range);
        nh_.param<double>("smoke_adaptive/score_weights/valid_count", smoke_cfg_.weight_valid_count, smoke_cfg_.weight_valid_count);
        nh_.param<double>("smoke_adaptive/score_weights/far_low_reflectivity", smoke_cfg_.weight_far_low_reflectivity, smoke_cfg_.weight_far_low_reflectivity);
        nh_.param<double>("smoke_adaptive/score_weights/far_isolation", smoke_cfg_.weight_far_isolation, smoke_cfg_.weight_far_isolation);
        nh_.param<double>("smoke_adaptive/livox/far_low_reflectivity", smoke_cfg_.far_low_reflectivity, smoke_cfg_.far_low_reflectivity);
        nh_.param<double>("smoke_adaptive/livox/min_valid_points_ratio", smoke_cfg_.min_valid_points_ratio, smoke_cfg_.min_valid_points_ratio);
        nh_.param<double>("smoke_adaptive/livox/isolated_voxel_leaf", smoke_cfg_.isolated_voxel_leaf, smoke_cfg_.isolated_voxel_leaf);
        nh_.param<int>("smoke_adaptive/livox/isolated_voxel_max_points", smoke_cfg_.isolated_voxel_max_points, smoke_cfg_.isolated_voxel_max_points);
        nh_.param<double>("smoke_adaptive/radar/min_power_normal", smoke_cfg_.radar_min_power_normal, smoke_cfg_.radar_min_power_normal);
        nh_.param<double>("smoke_adaptive/radar/min_power_smoke", smoke_cfg_.radar_min_power_smoke, smoke_cfg_.radar_min_power_smoke);
        nh_.param<int>("smoke_adaptive/radar/max_points_normal", smoke_cfg_.radar_max_points_normal, smoke_cfg_.radar_max_points_normal);
        nh_.param<int>("smoke_adaptive/radar/max_points_smoke", smoke_cfg_.radar_max_points_smoke, smoke_cfg_.radar_max_points_smoke);
        radar_queue_size_ = std::max(1, radar_queue_size_);
        radar_stale_time_ = std::max(max_sync_dt_, radar_stale_time_);
        fallback_livox_scan_duration_ = std::max(0.0, fallback_livox_scan_duration_);
        radar_ring_ = static_cast<std::uint16_t>(std::max(0, std::min(radar_ring, 65535)));
        normalizeSmokeConfig();

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

        ROS_INFO_STREAM("radar_livox_fusion_node started.\n"
                        << "  livox_topic: " << livox_topic_ << "\n"
                        << "  mmwave_topic: " << mmwave_topic_ << "\n"
                        << "  livox_raw_topic: " << livox_raw_topic_ << "\n"
                        << "  fusion_topic: " << fusion_topic_ << "\n"
                        << "  output_frame: " << output_frame_id_ << "\n"
                        << "  max_sync_dt: " << max_sync_dt_ << "\n"
                        << "  radar_queue_size: " << radar_queue_size_ << "\n"
                        << "  radar_stale_time: " << radar_stale_time_ << "\n"
                        << "  fallback_livox_scan_duration: " << fallback_livox_scan_duration_ << "\n"
                        << "  radar_ring: " << radar_ring_ << "\n"
                        << "  range_filter: " << (is_filter_ ? "enabled" : "disabled") << "\n"
                        << "  smoke_adaptive: " << (smoke_cfg_.enabled ? "enabled" : "disabled"));
    }

private:
    void mmwaveCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!radar_queue_.empty() && msg->header.stamp < radar_queue_.back()->header.stamp)
        {
            ROS_WARN_THROTTLE(1.0, "Radar timestamp rollback detected, clearing radar queue.");
            radar_queue_.clear();
        }

        radar_queue_.push_back(msg);
        while (static_cast<int>(radar_queue_.size()) > radar_queue_size_)
        {
            radar_queue_.pop_front();
            ++dropped_overflow_count_;
        }
    }

    void livoxCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg)
    {
        CloudXYZI::Ptr livox_raw_cloud(new CloudXYZI());
        CloudXYZIRT::Ptr fused_cloud(new CloudXYZIRT());
        std::vector<LivoxPointCandidate> livox_candidates;
        std::vector<RadarPointCandidate> radar_candidates;
        FrameQuality quality;

        buildLivoxRawCloud(msg, livox_raw_cloud);
        publishCloud(livox_raw_cloud, msg->header.stamp, output_frame_id_, pub_livox_raw_);

        double livox_scan_duration = 0.0;
        buildLivoxCandidates(msg, livox_candidates, quality, livox_scan_duration);
        if (livox_candidates.empty())
        {
            return;
        }
        if (livox_scan_duration <= 0.0)
        {
            livox_scan_duration = fallback_livox_scan_duration_;
        }

        sensor_msgs::PointCloud::ConstPtr mmwave_msg = takeBestMmwave(msg->header.stamp);
        if (mmwave_msg)
        {
            const double radar_relative_time = (mmwave_msg->header.stamp - msg->header.stamp).toSec();
            const float radar_point_time = static_cast<float>(clamp(radar_relative_time, 0.0, livox_scan_duration));
            buildRadarCandidates(mmwave_msg, radar_point_time, radar_candidates, quality);
        }

        updateSmokeMode(quality);
        appendFilteredLivox(livox_candidates, quality, fused_cloud);
        appendFilteredRadar(radar_candidates, quality, fused_cloud);

        if (fused_cloud->empty())
        {
            ROS_WARN_THROTTLE(1.0, "Smoke-adaptive filtering removed all fused points; publishing skipped.");
            return;
        }

        std::stable_sort(fused_cloud->begin(), fused_cloud->end(), [](const PointXYZIRT &a, const PointXYZIRT &b) {
            return a.time < b.time;
        });

        logSmokeDiagnostics(quality);
        publishTimedCloud(fused_cloud, msg->header.stamp, output_frame_id_, pub_fusion_);
    }

    sensor_msgs::PointCloud::ConstPtr takeBestMmwave(const ros::Time &livox_stamp)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (radar_queue_.empty())
        {
            ++livox_only_count_;
            ROS_INFO_THROTTLE(2.0, "No radar frame available, publishing Livox-only cloud.");
            return sensor_msgs::PointCloud::ConstPtr();
        }

        const double stale_before = livox_stamp.toSec() - radar_stale_time_;
        size_t stale_count = 0;
        while (!radar_queue_.empty() && radar_queue_.front()->header.stamp.toSec() < stale_before)
        {
            radar_queue_.pop_front();
            ++stale_count;
        }
        if (stale_count > 0)
        {
            dropped_stale_count_ += stale_count;
            ROS_WARN_THROTTLE(2.0, "Dropped %zu stale radar frames.", stale_count);
        }

        if (radar_queue_.empty())
        {
            ++livox_only_count_;
            ROS_INFO_THROTTLE(2.0, "No radar frame within stale window, publishing Livox-only cloud.");
            return sensor_msgs::PointCloud::ConstPtr();
        }

        auto best_it = radar_queue_.begin();
        double best_dt = std::fabs((livox_stamp - (*best_it)->header.stamp).toSec());
        for (auto it = radar_queue_.begin() + 1; it != radar_queue_.end(); ++it)
        {
            const double dt = std::fabs((livox_stamp - (*it)->header.stamp).toSec());
            if (dt < best_dt)
            {
                best_dt = dt;
                best_it = it;
            }
        }

        if (best_dt > max_sync_dt_)
        {
            ++livox_only_count_;
            ROS_INFO_THROTTLE(2.0, "No radar match within %.3f s, best dt %.3f s. Publishing Livox-only cloud.",
                              max_sync_dt_, best_dt);
            return sensor_msgs::PointCloud::ConstPtr();
        }

        sensor_msgs::PointCloud::ConstPtr best_msg = *best_it;
        radar_queue_.erase(best_it);
        ++matched_count_;
        ROS_INFO_THROTTLE(2.0, "Matched radar frame with dt %.6f s. matched=%llu livox_only=%llu stale_drop=%llu overflow_drop=%llu",
                          best_dt,
                          static_cast<unsigned long long>(matched_count_),
                          static_cast<unsigned long long>(livox_only_count_),
                          static_cast<unsigned long long>(dropped_stale_count_),
                          static_cast<unsigned long long>(dropped_overflow_count_));
        return best_msg;
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

    void buildLivoxCandidates(const livox_ros_driver::CustomMsg::ConstPtr &msg,
                              std::vector<LivoxPointCandidate> &out,
                              FrameQuality &quality,
                              double &scan_duration)
    {
        out.clear();
        out.reserve(msg->point_num);
        scan_duration = 0.0;
        std::map<std::tuple<int, int, int>, int> far_voxel_counts;

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

            PointXYZIRT q;
            q.x = p.x;
            q.y = p.y;
            q.z = p.z;
            q.intensity = static_cast<float>(p.reflectivity);
            q.time = static_cast<float>(static_cast<double>(p.offset_time) * 1.0e-9);
            q.ring = static_cast<std::uint16_t>(p.line);
            if (is_filter_ && !rangePass(q))
            {
                continue;
            }
            LivoxPointCandidate candidate;
            candidate.point = q;
            candidate.range = pointRange(q);
            candidate.reflectivity = q.intensity;

            ++quality.valid_livox_points;
            if (candidate.range >= smoke_cfg_.far_range)
            {
                ++quality.far_livox_points;
                if (candidate.reflectivity < smoke_cfg_.far_low_reflectivity)
                {
                    ++quality.far_low_reflectivity_points;
                }
                const auto key = voxelKey(q, smoke_cfg_.isolated_voxel_leaf);
                ++far_voxel_counts[key];
            }

            scan_duration = std::max(scan_duration, static_cast<double>(q.time));
            out.push_back(candidate);
        }

        quality.far_voxels = far_voxel_counts.size();
        for (const auto &kv : far_voxel_counts)
        {
            if (kv.second <= smoke_cfg_.isolated_voxel_max_points)
            {
                ++quality.far_isolated_voxels;
            }
        }
        for (auto &candidate : out)
        {
            if (candidate.range >= smoke_cfg_.far_range)
            {
                const auto key = voxelKey(candidate.point, smoke_cfg_.isolated_voxel_leaf);
                candidate.far_voxel_count = far_voxel_counts[key];
            }
        }
    }

    void buildRadarCandidates(const sensor_msgs::PointCloud::ConstPtr &msg,
                              const float point_time,
                              std::vector<RadarPointCandidate> &out,
                              FrameQuality &quality)
    {
        out.clear();
        out.reserve(msg->points.size());

        const int intensity_channel = findChannel(*msg, {"Power", "intensity"});

        for (size_t i = 0; i < msg->points.size(); ++i)
        {
            PointXYZIRT q;
            q.x = msg->points[i].x;
            q.y = msg->points[i].y;
            q.z = msg->points[i].z;
            q.intensity = 1.0f;
            q.time = point_time;
            q.ring = radar_ring_;

            if (intensity_channel >= 0 && i < msg->channels[intensity_channel].values.size())
            {
                q.intensity = msg->channels[intensity_channel].values[i];
            }

            if (is_filter_ && !rangePass(q))
            {
                continue;
            }

            RadarPointCandidate candidate;
            candidate.point = q;
            candidate.range = pointRange(q);
            candidate.power = q.intensity;
            out.push_back(candidate);
        }
        quality.radar_points_before = out.size();
    }

    void transformMmwaveToLivox(CloudXYZIRT::Ptr &cloud)
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

    void transformMmwavePointToLivox(PointXYZIRT &point)
    {
        Eigen::Vector3d p_m(point.x, point.y, point.z);
        Eigen::Vector3d p_l = r_m2l_ * p_m + t_m2l_;
        point.x = static_cast<float>(p_l.x());
        point.y = static_cast<float>(p_l.y());
        point.z = static_cast<float>(p_l.z());
    }

    void normalizeSmokeConfig()
    {
        smoke_cfg_.enter_frames = std::max(1, smoke_cfg_.enter_frames);
        smoke_cfg_.exit_frames = std::max(1, smoke_cfg_.exit_frames);
        smoke_cfg_.enter_score = clamp(smoke_cfg_.enter_score, 0.0, 1.0);
        smoke_cfg_.exit_score = clamp(smoke_cfg_.exit_score, 0.0, smoke_cfg_.enter_score);
        smoke_cfg_.near_range = std::max(0.0, smoke_cfg_.near_range);
        smoke_cfg_.far_range = std::max(smoke_cfg_.near_range, smoke_cfg_.far_range);
        smoke_cfg_.min_valid_points_ratio = clamp(smoke_cfg_.min_valid_points_ratio, 0.01, 1.0);
        smoke_cfg_.isolated_voxel_leaf = std::max(0.05, smoke_cfg_.isolated_voxel_leaf);
        smoke_cfg_.isolated_voxel_max_points = std::max(1, smoke_cfg_.isolated_voxel_max_points);
        smoke_cfg_.radar_max_points_normal = std::max(0, smoke_cfg_.radar_max_points_normal);
        smoke_cfg_.radar_max_points_smoke = std::max(smoke_cfg_.radar_max_points_normal, smoke_cfg_.radar_max_points_smoke);

        const double weight_sum = smoke_cfg_.weight_valid_count +
                                  smoke_cfg_.weight_far_low_reflectivity +
                                  smoke_cfg_.weight_far_isolation;
        if (weight_sum <= 1.0e-6)
        {
            smoke_cfg_.weight_valid_count = 0.35;
            smoke_cfg_.weight_far_low_reflectivity = 0.35;
            smoke_cfg_.weight_far_isolation = 0.30;
        }
    }

    void updateSmokeMode(FrameQuality &quality)
    {
        if (baseline_valid_livox_points_ <= 1.0)
        {
            baseline_valid_livox_points_ = static_cast<double>(std::max<size_t>(1, quality.valid_livox_points));
        }
        quality.baseline_valid_livox_points = baseline_valid_livox_points_;
        quality.valid_points_ratio = static_cast<double>(quality.valid_livox_points) / std::max(1.0, baseline_valid_livox_points_);
        quality.far_low_reflectivity_ratio = static_cast<double>(quality.far_low_reflectivity_points) /
                                             static_cast<double>(std::max<size_t>(1, quality.far_livox_points));
        quality.far_isolated_voxel_ratio = static_cast<double>(quality.far_isolated_voxels) /
                                           static_cast<double>(std::max<size_t>(1, quality.far_voxels));
        quality.low_valid_point_score = clamp((smoke_cfg_.min_valid_points_ratio - quality.valid_points_ratio) /
                                                  smoke_cfg_.min_valid_points_ratio,
                                              0.0,
                                              1.0);
        quality.far_low_reflectivity_score = clamp(quality.far_low_reflectivity_ratio, 0.0, 1.0);
        quality.far_isolated_voxel_score = clamp(quality.far_isolated_voxel_ratio, 0.0, 1.0);

        const double weight_sum = std::max(1.0e-6,
                                           smoke_cfg_.weight_valid_count +
                                               smoke_cfg_.weight_far_low_reflectivity +
                                               smoke_cfg_.weight_far_isolation);
        quality.smoke_score = clamp((smoke_cfg_.weight_valid_count * quality.low_valid_point_score +
                                     smoke_cfg_.weight_far_low_reflectivity * quality.far_low_reflectivity_score +
                                     smoke_cfg_.weight_far_isolation * quality.far_isolated_voxel_score) /
                                        weight_sum,
                                    0.0,
                                    1.0);

        if (!smoke_cfg_.enabled)
        {
            environment_mode_ = EnvironmentMode::NORMAL;
            enter_smoke_count_ = 0;
            exit_smoke_count_ = 0;
        }
        else if (quality.smoke_score >= smoke_cfg_.enter_score)
        {
            ++enter_smoke_count_;
            exit_smoke_count_ = 0;
            if (enter_smoke_count_ >= smoke_cfg_.enter_frames)
            {
                environment_mode_ = EnvironmentMode::SMOKE;
            }
            else if (environment_mode_ == EnvironmentMode::NORMAL)
            {
                environment_mode_ = EnvironmentMode::SMOKE_SUSPECTED;
            }
        }
        else if (quality.smoke_score <= smoke_cfg_.exit_score)
        {
            ++exit_smoke_count_;
            enter_smoke_count_ = 0;
            if (exit_smoke_count_ >= smoke_cfg_.exit_frames || environment_mode_ == EnvironmentMode::SMOKE_SUSPECTED)
            {
                environment_mode_ = EnvironmentMode::NORMAL;
            }
        }
        else
        {
            enter_smoke_count_ = 0;
            exit_smoke_count_ = 0;
            if (environment_mode_ == EnvironmentMode::NORMAL)
            {
                environment_mode_ = EnvironmentMode::SMOKE_SUSPECTED;
            }
        }

        if (environment_mode_ == EnvironmentMode::NORMAL)
        {
            const double current_valid = static_cast<double>(std::max<size_t>(1, quality.valid_livox_points));
            baseline_valid_livox_points_ = 0.98 * baseline_valid_livox_points_ + 0.02 * current_valid;
        }
        quality.mode = environment_mode_;
    }

    void appendFilteredLivox(const std::vector<LivoxPointCandidate> &candidates,
                             FrameQuality &quality,
                             CloudXYZIRT::Ptr &out)
    {
        for (const auto &candidate : candidates)
        {
            bool drop = false;
            const bool far_point = candidate.range >= smoke_cfg_.far_range;
            const bool low_reflectivity = candidate.reflectivity < smoke_cfg_.far_low_reflectivity;
            const bool isolated = candidate.far_voxel_count > 0 &&
                                  candidate.far_voxel_count <= smoke_cfg_.isolated_voxel_max_points;

            if (smoke_cfg_.enabled && far_point)
            {
                if (quality.mode == EnvironmentMode::SMOKE_SUSPECTED)
                {
                    drop = low_reflectivity && isolated;
                }
                else if (quality.mode == EnvironmentMode::SMOKE)
                {
                    drop = low_reflectivity || isolated;
                }
            }

            if (!drop)
            {
                out->push_back(candidate.point);
            }
        }
        quality.livox_points_after = out->size();
    }

    void appendFilteredRadar(const std::vector<RadarPointCandidate> &candidates,
                             FrameQuality &quality,
                             CloudXYZIRT::Ptr &out)
    {
        if (candidates.empty())
        {
            quality.radar_points_after = 0;
            return;
        }

        double min_power = smoke_cfg_.radar_min_power_normal;
        int max_points = smoke_cfg_.radar_max_points_normal;
        if (!smoke_cfg_.enabled)
        {
            min_power = -1.0e9;
            max_points = static_cast<int>(candidates.size());
        }
        else if (quality.mode == EnvironmentMode::SMOKE)
        {
            min_power = smoke_cfg_.radar_min_power_smoke;
            max_points = smoke_cfg_.radar_max_points_smoke;
        }
        else if (quality.mode == EnvironmentMode::SMOKE_SUSPECTED)
        {
            min_power = 0.5 * (smoke_cfg_.radar_min_power_normal + smoke_cfg_.radar_min_power_smoke);
            max_points = std::max(smoke_cfg_.radar_max_points_normal,
                                  (smoke_cfg_.radar_max_points_normal + smoke_cfg_.radar_max_points_smoke) / 2);
        }

        std::vector<RadarPointCandidate> selected;
        selected.reserve(candidates.size());
        for (const auto &candidate : candidates)
        {
            if (candidate.power >= min_power)
            {
                selected.push_back(candidate);
            }
        }

        std::stable_sort(selected.begin(), selected.end(), [](const RadarPointCandidate &a, const RadarPointCandidate &b) {
            return a.power > b.power;
        });
        if (max_points >= 0 && static_cast<int>(selected.size()) > max_points)
        {
            selected.resize(static_cast<size_t>(max_points));
        }

        for (auto candidate : selected)
        {
            transformMmwavePointToLivox(candidate.point);
            out->push_back(candidate.point);
        }
        quality.radar_points_after = selected.size();
    }

    void logSmokeDiagnostics(const FrameQuality &quality) const
    {
        if (!smoke_cfg_.enabled)
        {
            return;
        }

        ROS_INFO_THROTTLE(2.0,
                          "smoke_adaptive mode=%s score=%.3f components(valid=%.3f refl=%.3f isolated=%.3f) "
                          "livox(valid=%zu base=%.1f far=%zu far_low_ratio=%.3f far_iso_ratio=%.3f after=%zu) "
                          "radar(before=%zu after=%zu)",
                          modeName(quality.mode).c_str(),
                          quality.smoke_score,
                          quality.low_valid_point_score,
                          quality.far_low_reflectivity_score,
                          quality.far_isolated_voxel_score,
                          quality.valid_livox_points,
                          quality.baseline_valid_livox_points,
                          quality.far_livox_points,
                          quality.far_low_reflectivity_ratio,
                          quality.far_isolated_voxel_ratio,
                          quality.livox_points_after,
                          quality.radar_points_before,
                          quality.radar_points_after);
    }

    std::string modeName(const EnvironmentMode mode) const
    {
        switch (mode)
        {
        case EnvironmentMode::NORMAL:
            return "normal";
        case EnvironmentMode::SMOKE_SUSPECTED:
            return "smoke_suspected";
        case EnvironmentMode::SMOKE:
            return "smoke";
        }
        return "unknown";
    }

    template <typename PointT>
    double pointRange(const PointT &point) const
    {
        return std::sqrt(static_cast<double>(point.x) * point.x +
                         static_cast<double>(point.y) * point.y +
                         static_cast<double>(point.z) * point.z);
    }

    template <typename PointT>
    std::tuple<int, int, int> voxelKey(const PointT &point, const double leaf) const
    {
        const double safe_leaf = std::max(0.05, leaf);
        return std::make_tuple(static_cast<int>(std::floor(point.x / safe_leaf)),
                               static_cast<int>(std::floor(point.y / safe_leaf)),
                               static_cast<int>(std::floor(point.z / safe_leaf)));
    }

    double clamp(const double value, const double min_value, const double max_value) const
    {
        return std::max(min_value, std::min(value, max_value));
    }

    int findChannel(const sensor_msgs::PointCloud &msg, const std::vector<std::string> &names) const
    {
        for (const auto &name : names)
        {
            for (size_t channel_id = 0; channel_id < msg.channels.size(); ++channel_id)
            {
                if (msg.channels[channel_id].name == name)
                {
                    return static_cast<int>(channel_id);
                }
            }
        }
        return -1;
    }

    template <typename PointT>
    bool rangePass(const PointT &point) const
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

    void publishTimedCloud(const CloudXYZIRT::Ptr &cloud,
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
    double fallback_livox_scan_duration_ = 0.1;
    double max_sync_dt_ = 0.03;
    int radar_queue_size_ = 50;
    double radar_stale_time_ = 0.2;
    std::uint16_t radar_ring_ = 0;
    bool is_filter_ = true;
    double min_range_ = 1.0;
    double max_range_ = 120.0;
    double voxel_leaf_ = 0.2;
    SmokeAdaptiveConfig smoke_cfg_;
    EnvironmentMode environment_mode_ = EnvironmentMode::NORMAL;
    int enter_smoke_count_ = 0;
    int exit_smoke_count_ = 0;
    double baseline_valid_livox_points_ = 0.0;

    Eigen::Matrix3d r_m2l_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_m2l_ = Eigen::Vector3d::Zero();

    std::mutex mutex_;
    std::deque<sensor_msgs::PointCloud::ConstPtr> radar_queue_;
    uint64_t matched_count_ = 0;
    uint64_t livox_only_count_ = 0;
    uint64_t dropped_stale_count_ = 0;
    uint64_t dropped_overflow_count_ = 0;
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
