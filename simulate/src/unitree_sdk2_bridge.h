#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include <array>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include "param.h"
#include "physics_joystick.h"

#define MOTOR_SENSOR_NUM 3

class UnitreeSDK2BridgeBase
{
public:
    UnitreeSDK2BridgeBase(mjModel *model, mjData *data)
    : mj_model_(model), mj_data_(data)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

    }

    virtual void start() {}
    virtual void on_after_physics_step() {}

    bool get_published_soccer_targets(std::array<float, 3> & ball_pos_w, std::array<float, 3> & goal_pos_w) const
    {
        std::lock_guard<std::mutex> lock(soccer_targets_mutex_);
        if (!published_soccer_targets_valid_) {
            return false;
        }
        ball_pos_w = published_soccer_ball_pos_w_;
        goal_pos_w = published_soccer_goal_pos_w_;
        return true;
    }

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };
    
        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
    
        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;

    // Sensor data indices
    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;
    int frame_pos_adr_ = -1;
    int frame_vel_adr_ = -1;

    int secondary_imu_quat_adr_ = -1;
    int secondary_imu_gyro_adr_ = -1;
    int secondary_imu_acc_adr_ = -1;
    mutable std::mutex soccer_state_mutex_;
    mutable std::mutex soccer_targets_mutex_;
    std::array<float, 3> published_soccer_ball_pos_w_ = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> published_soccer_goal_pos_w_ = {0.0f, 0.0f, 0.0f};
    bool published_soccer_targets_valid_ = false;
    std::vector<int> soccer_ball_body_ids_;
    int active_soccer_ball_body_id_ = -1;
    bool soccer_goal_initialized_ = false;
    std::array<float, 3> soccer_goal_pos_w_ = {0.0f, 0.0f, 0.0f};
    bool soccer_missing_warned_ = false;
    std::mt19937 soccer_rng_{std::random_device{}()};

    std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

    void _check_sensor()
    {
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;
    
        // Find sensor addresses by name
        int sensor_id = -1;
        
        // IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
        if (sensor_id >= 0) {
            imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_gyro");
        if (sensor_id >= 0) {
            imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_acc");
        if (sensor_id >= 0) {
            imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame position
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_pos");
        if (sensor_id >= 0) {
            frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame velocity
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_vel");
        if (sensor_id >= 0) {
            frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_quat");
        if (sensor_id >= 0) {
            secondary_imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_gyro");
        if (sensor_id >= 0) {
            secondary_imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_acc");
        if (sensor_id >= 0) {
            secondary_imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        _check_soccer_bodies();
    }

    void _check_soccer_bodies()
    {
        soccer_ball_body_ids_.clear();
        for (int i = 0; i < mj_model_->nbody; ++i) {
            const char* body_name = mj_id2name(mj_model_, mjOBJ_BODY, i);
            if (body_name && std::string(body_name).find("soccer_ball") != std::string::npos) {
                soccer_ball_body_ids_.push_back(i);
            }
        }
        if (soccer_ball_body_ids_.empty()) {
            std::cout << "[unitree_mujoco] No `soccer_ball*` bodies found in scene. "
                      << "Stage2 ball/goal targets will not be published." << std::endl;
            soccer_missing_warned_ = true;
        } else {
            std::cout << "[unitree_mujoco] Soccer targets enabled with "
                      << soccer_ball_body_ids_.size() << " candidate ball bodies." << std::endl;
            soccer_missing_warned_ = false;
        }
    }

    void _set_published_soccer_targets(const std::array<float, 3> & ball_pos_w, const std::array<float, 3> & goal_pos_w)
    {
        std::lock_guard<std::mutex> lock(soccer_targets_mutex_);
        published_soccer_ball_pos_w_ = ball_pos_w;
        published_soccer_goal_pos_w_ = goal_pos_w;
        published_soccer_targets_valid_ = true;
    }

    void _set_published_soccer_targets_invalid()
    {
        std::lock_guard<std::mutex> lock(soccer_targets_mutex_);
        published_soccer_targets_valid_ = false;
    }

    bool _query_soccer_world_targets(std::array<float, 3> & ball_pos_w, std::array<float, 3> & goal_pos_w)
    {
        std::lock_guard<std::mutex> soccer_lock(soccer_state_mutex_);
        if (!mj_data_ || soccer_ball_body_ids_.empty() || frame_pos_adr_ < 0 || imu_quat_adr_ < 0) {
            if (!soccer_missing_warned_ && soccer_ball_body_ids_.empty()) {
                std::cout << "[unitree_mujoco] Soccer targets unavailable: no `soccer_ball*` bodies." << std::endl;
                soccer_missing_warned_ = true;
            }
            return false;
        }

        const float robot_x = static_cast<float>(mj_data_->sensordata[frame_pos_adr_ + 0]);
        const float robot_y = static_cast<float>(mj_data_->sensordata[frame_pos_adr_ + 1]);

        int nearest_ball_body_id = -1;
        double nearest_dist_sq = std::numeric_limits<double>::infinity();
        std::array<float, 3> nearest_ball_pos_w{0.0f, 0.0f, 0.0f};
        std::array<float, 3> active_ball_pos_w{0.0f, 0.0f, 0.0f};
        double active_dist_sq = std::numeric_limits<double>::infinity();
        bool active_valid = false;

        for (int body_id : soccer_ball_body_ids_) {
            const int adr = 3 * body_id;
            const double x = mj_data_->xpos[adr + 0];
            const double y = mj_data_->xpos[adr + 1];
            const double z = mj_data_->xpos[adr + 2];
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                continue;
            }
            const double dx = x - static_cast<double>(robot_x);
            const double dy = y - static_cast<double>(robot_y);
            const double dist_sq = dx * dx + dy * dy;
            if (dist_sq < nearest_dist_sq) {
                nearest_dist_sq = dist_sq;
                nearest_ball_body_id = body_id;
                nearest_ball_pos_w = {
                    static_cast<float>(x),
                    static_cast<float>(y),
                    static_cast<float>(z),
                };
            }
            if (body_id == active_soccer_ball_body_id_) {
                active_valid = true;
                active_dist_sq = dist_sq;
                active_ball_pos_w = {
                    static_cast<float>(x),
                    static_cast<float>(y),
                    static_cast<float>(z),
                };
            }
        }

        if (nearest_ball_body_id < 0) {
            return false;
        }

        int best_ball_body_id = nearest_ball_body_id;
        std::array<float, 3> best_ball_pos_w = nearest_ball_pos_w;

        // Keep the previously selected ball unless another candidate is clearly closer.
        constexpr double kSwitchMarginMeters = 0.20;
        const double switch_margin_sq = kSwitchMarginMeters * kSwitchMarginMeters;
        if (active_valid && (nearest_dist_sq + switch_margin_sq) >= active_dist_sq) {
            best_ball_body_id = active_soccer_ball_body_id_;
            best_ball_pos_w = active_ball_pos_w;
        }

        const float w = static_cast<float>(mj_data_->sensordata[imu_quat_adr_ + 0]);
        const float x = static_cast<float>(mj_data_->sensordata[imu_quat_adr_ + 1]);
        const float y = static_cast<float>(mj_data_->sensordata[imu_quat_adr_ + 2]);
        const float z = static_cast<float>(mj_data_->sensordata[imu_quat_adr_ + 3]);
        const float yaw = std::atan2(
            2.0f * (w * z + x * y),
            1.0f - 2.0f * (y * y + z * z)
        );

        constexpr float kGoalForwardDistance = 5.0f;
        if (!soccer_goal_initialized_ || best_ball_body_id != active_soccer_ball_body_id_) {
            soccer_goal_pos_w_ = {
                robot_x + kGoalForwardDistance * std::cos(yaw),
                robot_y + kGoalForwardDistance * std::sin(yaw),
                0.0f,
            };
            soccer_goal_initialized_ = true;
            active_soccer_ball_body_id_ = best_ball_body_id;
        }

        goal_pos_w = soccer_goal_pos_w_;
        ball_pos_w = best_ball_pos_w;
        return true;
    }

    bool _randomize_soccer_ball_on_stage2_enter(bool rolling_ball_on_enter)
    {
        if (!param::config.stage2_random_ball_on_enter) {
            return false;
        }
        std::lock_guard<std::mutex> soccer_lock(soccer_state_mutex_);
        if (!mj_data_ || soccer_ball_body_ids_.empty() || frame_pos_adr_ < 0 || imu_quat_adr_ < 0) {
            return false;
        }

        const float robot_x = static_cast<float>(mj_data_->sensordata[frame_pos_adr_ + 0]);
        const float robot_y = static_cast<float>(mj_data_->sensordata[frame_pos_adr_ + 1]);

        int selected_ball_body_id = -1;
        std::array<float, 3> selected_ball_pos_w{0.0f, 0.0f, 0.0f};
        bool selected_from_active = false;
        if (active_soccer_ball_body_id_ >= 0 &&
            std::find(soccer_ball_body_ids_.begin(), soccer_ball_body_ids_.end(), active_soccer_ball_body_id_) != soccer_ball_body_ids_.end()) {
            const int adr = 3 * active_soccer_ball_body_id_;
            const double x = mj_data_->xpos[adr + 0];
            const double y = mj_data_->xpos[adr + 1];
            const double z = mj_data_->xpos[adr + 2];
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                selected_ball_body_id = active_soccer_ball_body_id_;
                selected_ball_pos_w = {
                    static_cast<float>(x),
                    static_cast<float>(y),
                    static_cast<float>(z),
                };
                selected_from_active = true;
            }
        }

        if (!selected_from_active) {
            double nearest_dist_sq = std::numeric_limits<double>::infinity();
            for (int body_id : soccer_ball_body_ids_) {
                const int adr = 3 * body_id;
                const double x = mj_data_->xpos[adr + 0];
                const double y = mj_data_->xpos[adr + 1];
                const double z = mj_data_->xpos[adr + 2];
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                    continue;
                }
                const double dx = x - static_cast<double>(robot_x);
                const double dy = y - static_cast<double>(robot_y);
                const double dist_sq = dx * dx + dy * dy;
                if (dist_sq < nearest_dist_sq) {
                    nearest_dist_sq = dist_sq;
                    selected_ball_body_id = body_id;
                    selected_ball_pos_w = {
                        static_cast<float>(x),
                        static_cast<float>(y),
                        static_cast<float>(z),
                    };
                }
            }
        }

        if (selected_ball_body_id < 0) {
            return false;
        }

        if (mj_model_->body_jntnum[selected_ball_body_id] <= 0) {
            return false;
        }
        const int joint_id = mj_model_->body_jntadr[selected_ball_body_id];
        if (joint_id < 0 || mj_model_->jnt_type[joint_id] != mjJNT_FREE) {
            return false;
        }
        const int qadr = mj_model_->jnt_qposadr[joint_id];
        const int vadr = mj_model_->jnt_dofadr[joint_id];
        if (qadr < 0 || vadr < 0) {
            return false;
        }

        const float w = static_cast<float>(mj_data_->sensordata[imu_quat_adr_ + 0]);
        const float x = static_cast<float>(mj_data_->sensordata[imu_quat_adr_ + 1]);
        const float y = static_cast<float>(mj_data_->sensordata[imu_quat_adr_ + 2]);
        const float z = static_cast<float>(mj_data_->sensordata[imu_quat_adr_ + 3]);
        const float yaw = std::atan2(
            2.0f * (w * z + x * y),
            1.0f - 2.0f * (y * y + z * z)
        );
        const float cy = std::cos(yaw);
        const float sy = std::sin(yaw);

        const float rel_x_w = selected_ball_pos_w[0] - robot_x;
        const float rel_y_w = selected_ball_pos_w[1] - robot_y;
        const float nominal_x_b = cy * rel_x_w + sy * rel_y_w;
        const float nominal_y_b = -sy * rel_x_w + cy * rel_y_w;
        float nominal_radius = std::hypot(nominal_x_b, nominal_y_b);
        if (!std::isfinite(nominal_radius) || nominal_radius < 0.2f) {
            nominal_radius = 1.0f;
        }
        float nominal_angle = std::atan2(nominal_y_b, nominal_x_b);
        if (!std::isfinite(nominal_angle)) {
            nominal_angle = 0.0f;
        }

        const auto angle_range = param::config.stage2_spawn_angle_range;
        const auto radius_range = param::config.stage2_spawn_radius_range;
        std::uniform_real_distribution<float> dist_angle(
            std::min(angle_range[0], angle_range[1]),
            std::max(angle_range[0], angle_range[1])
        );
        std::uniform_real_distribution<float> dist_radius_noise(
            std::min(radius_range[0], radius_range[1]),
            std::max(radius_range[0], radius_range[1])
        );

        const float angle = nominal_angle + dist_angle(soccer_rng_);
        float radius = nominal_radius + dist_radius_noise(soccer_rng_);
        radius = std::max(radius, std::max(0.0f, param::config.stage2_min_ball_distance));
        float local_x = radius * std::cos(angle);
        const float local_y = radius * std::sin(angle);
        if (param::config.stage2_enforce_forward_spawn) {
            local_x = std::max(local_x, param::config.stage2_min_forward_spawn_x);
        }

        const float world_x = robot_x + cy * local_x - sy * local_y;
        const float world_y = robot_y + sy * local_x + cy * local_y;
        const float world_z = std::isfinite(selected_ball_pos_w[2]) ? selected_ball_pos_w[2] : param::config.stage2_ball_height;

        mj_data_->qpos[qadr + 0] = world_x;
        mj_data_->qpos[qadr + 1] = world_y;
        mj_data_->qpos[qadr + 2] = world_z;
        mj_data_->qpos[qadr + 3] = 1.0;
        mj_data_->qpos[qadr + 4] = 0.0;
        mj_data_->qpos[qadr + 5] = 0.0;
        mj_data_->qpos[qadr + 6] = 0.0;

        for (int i = 0; i < 6; ++i) {
            mj_data_->qvel[vadr + i] = 0.0;
        }

        if (rolling_ball_on_enter) {
            const auto speed_range = param::config.stage2_rolling_speed_range;
            const float speed_min = std::max(0.0f, std::min(speed_range[0], speed_range[1]));
            const float speed_max = std::max(speed_min, std::max(speed_range[0], speed_range[1]));
            std::uniform_real_distribution<float> dist_speed(speed_min, speed_max);
            const float speed = dist_speed(soccer_rng_);
            const float radial_norm = std::max(1.0e-6f, std::hypot(local_x, local_y));
            float tx = -local_y / radial_norm;
            float ty = local_x / radial_norm;
            std::uniform_int_distribution<int> dist_sign(0, 1);
            if (dist_sign(soccer_rng_) == 0) {
                tx = -tx;
                ty = -ty;
            }
            const float world_vx = cy * tx - sy * ty;
            const float world_vy = sy * tx + cy * ty;
            mj_data_->qvel[vadr + 0] = world_vx * speed;
            mj_data_->qvel[vadr + 1] = world_vy * speed;
        }

        active_soccer_ball_body_id_ = selected_ball_body_id;
        soccer_goal_initialized_ = false;
        _set_published_soccer_targets_invalid();
        mj_forward(mj_model_, mj_data_);

        std::cout << "[unitree_mujoco] Stage2 random ball reset: body=" << selected_ball_body_id
                  << ", local=(" << local_x << ", " << local_y << "), rolling="
                  << (rolling_ball_on_enter ? "on" : "off") << std::endl;
        return true;
    }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase
{
using HighState_t = unitree::robot::go2::publisher::SportModeState;
using WirelessController_t = unitree::robot::go2::publisher::WirelessController;

public:
    RobotBridge(mjModel *model, mjData *data) : UnitreeSDK2BridgeBase(model, data)
    {
        lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd");
        lowstate = std::make_unique<LowState_t>();
        lowstate->joystick = joystick;
        highstate = std::make_unique<HighState_t>();
        wireless_controller = std::make_unique<WirelessController_t>();
        wireless_controller->joystick = joystick;
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "unitree_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
    }

    virtual void run()
    {
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }
        // lowcmd
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            for(int i(0); i<num_motor_; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }
        }

        // lowstate
        if(lowstate->trylock()) {
            for(int i(0); i<num_motor_; i++) {
                lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
                lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
                lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
            }
            
            if(imu_quat_adr_ >= 0) {
                lowstate->msg_.imu_state().quaternion()[0] = mj_data_->sensordata[imu_quat_adr_ + 0];
                lowstate->msg_.imu_state().quaternion()[1] = mj_data_->sensordata[imu_quat_adr_ + 1];
                lowstate->msg_.imu_state().quaternion()[2] = mj_data_->sensordata[imu_quat_adr_ + 2];
                lowstate->msg_.imu_state().quaternion()[3] = mj_data_->sensordata[imu_quat_adr_ + 3];

                double w = lowstate->msg_.imu_state().quaternion()[0];
                double x = lowstate->msg_.imu_state().quaternion()[1];
                double y = lowstate->msg_.imu_state().quaternion()[2];
                double z = lowstate->msg_.imu_state().quaternion()[3];

                lowstate->msg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
                lowstate->msg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }
            
            if(imu_gyro_adr_ >= 0) {
                lowstate->msg_.imu_state().gyroscope()[0] = mj_data_->sensordata[imu_gyro_adr_ + 0];
                lowstate->msg_.imu_state().gyroscope()[1] = mj_data_->sensordata[imu_gyro_adr_ + 1];
                lowstate->msg_.imu_state().gyroscope()[2] = mj_data_->sensordata[imu_gyro_adr_ + 2];
            }

            if(imu_acc_adr_ >= 0) {
                lowstate->msg_.imu_state().accelerometer()[0] = mj_data_->sensordata[imu_acc_adr_ + 0];
                lowstate->msg_.imu_state().accelerometer()[1] = mj_data_->sensordata[imu_acc_adr_ + 1];
                lowstate->msg_.imu_state().accelerometer()[2] = mj_data_->sensordata[imu_acc_adr_ + 2];
            }
            
            lowstate->msg_.tick() = std::round(mj_data_->time / 1e-3);
            lowstate->unlockAndPublish();
        }
        // highstate
        if(highstate->trylock()) {
            if(frame_pos_adr_ >= 0) {
                highstate->msg_.position()[0] = mj_data_->sensordata[frame_pos_adr_ + 0];
                highstate->msg_.position()[1] = mj_data_->sensordata[frame_pos_adr_ + 1];
                highstate->msg_.position()[2] = mj_data_->sensordata[frame_pos_adr_ + 2];
            }
            if(frame_vel_adr_ >= 0) {
                highstate->msg_.velocity()[0] = mj_data_->sensordata[frame_vel_adr_ + 0];
                highstate->msg_.velocity()[1] = mj_data_->sensordata[frame_vel_adr_ + 1];
                highstate->msg_.velocity()[2] = mj_data_->sensordata[frame_vel_adr_ + 2];
            }
            std::array<float, 3> ball_pos_w{0.0f, 0.0f, 0.0f};
            std::array<float, 3> goal_pos_w{0.0f, 0.0f, 0.0f};
            auto & path_points = highstate->msg_.path_point();
            if (_query_soccer_world_targets(ball_pos_w, goal_pos_w)) {
                path_points[0].t_from_start() = 1.0f;
                path_points[0].x() = ball_pos_w[0];
                path_points[0].y() = ball_pos_w[1];
                path_points[0].yaw() = ball_pos_w[2];

                path_points[1].t_from_start() = 1.0f;
                path_points[1].x() = goal_pos_w[0];
                path_points[1].y() = goal_pos_w[1];
                path_points[1].yaw() = goal_pos_w[2];
                _set_published_soccer_targets(ball_pos_w, goal_pos_w);
            } else {
                path_points[0].t_from_start() = 0.0f;
                path_points[1].t_from_start() = 0.0f;
                _set_published_soccer_targets_invalid();
            }
            highstate->unlockAndPublish();
        }
        // wireless_controller
        if(wireless_controller->joystick) {
            wireless_controller->unlockAndPublish();
        }
    }

    std::unique_ptr<HighState_t> highstate;
    std::unique_ptr<WirelessController_t> wireless_controller;
    std::shared_ptr<LowCmd_t> lowcmd;
    std::unique_ptr<LowState_t> lowstate;
    
private:
    unitree::common::RecurrentThreadPtr thread_;
};

using Go2Bridge = RobotBridge<unitree::robot::go2::subscription::LowCmd, unitree::robot::go2::publisher::LowState>;

class G1Bridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd, unitree::robot::g1::publisher::LowState>
{
public:
    G1Bridge(mjModel *model, mjData *data) : RobotBridge(model, data)
    {
        if (param::config.robot.find("g1") != std::string::npos) {
            auto* g1_lowstate = dynamic_cast<unitree::robot::g1::publisher::LowState*>(lowstate.get());
            if (g1_lowstate) {
                auto scene = param::config.robot_scene.filename().string();
                g1_lowstate->msg_.mode_machine() = scene.find("23") != std::string::npos ? 4 : 5;
            }
        }

        bmsstate = std::make_unique<BmsState_t>("rt/lf/bmsstate");
        bmsstate->msg_.soc() = 100;

        secondary_imustate = std::make_unique<IMUState_t>("rt/secondary_imu");
    }

    void run() override
    {
        RobotBridge::run();

        // secondary IMU state
        if (secondary_imustate->trylock()) {
            if(secondary_imu_quat_adr_ >= 0) {
                secondary_imustate->msg_.quaternion()[0] = mj_data_->sensordata[secondary_imu_quat_adr_ + 0];
                secondary_imustate->msg_.quaternion()[1] = mj_data_->sensordata[secondary_imu_quat_adr_ + 1];
                secondary_imustate->msg_.quaternion()[2] = mj_data_->sensordata[secondary_imu_quat_adr_ + 2];
                secondary_imustate->msg_.quaternion()[3] = mj_data_->sensordata[secondary_imu_quat_adr_ + 3];

                double w = secondary_imustate->msg_.quaternion()[0];
                double x = secondary_imustate->msg_.quaternion()[1];
                double y = secondary_imustate->msg_.quaternion()[2];
                double z = secondary_imustate->msg_.quaternion()[3];

                secondary_imustate->msg_.rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                secondary_imustate->msg_.rpy()[1] = asin(2 * (w * y - z * x));
                secondary_imustate->msg_.rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }

            if(secondary_imu_gyro_adr_ >= 0) {
                secondary_imustate->msg_.gyroscope()[0] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 0];
                secondary_imustate->msg_.gyroscope()[1] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 1];
                secondary_imustate->msg_.gyroscope()[2] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 2];
            }

            if(secondary_imu_acc_adr_ >= 0) {
                secondary_imustate->msg_.accelerometer()[0] = mj_data_->sensordata[secondary_imu_acc_adr_ + 0];
                secondary_imustate->msg_.accelerometer()[1] = mj_data_->sensordata[secondary_imu_acc_adr_ + 1];
                secondary_imustate->msg_.accelerometer()[2] = mj_data_->sensordata[secondary_imu_acc_adr_ + 2];
            }

            secondary_imustate->unlockAndPublish();
        }

        // In practice, bmsstate is sent at a low frequency; here it is sent with the main loop
        bmsstate->unlockAndPublish();
    }

    void on_after_physics_step() override
    {
        if (!lowcmd || !param::config.stage2_random_ball_on_enter) {
            return;
        }
        uint32_t magic = 0;
        uint32_t nonce = 0;
        uint32_t rolling = 0;
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            const auto & reserve = lowcmd->msg_.reserve();
            magic = reserve[0];
            nonce = reserve[1];
            rolling = reserve[2];
        }
        if (magic != kStage2SpawnMagic) {
            return;
        }
        if (stage2_spawn_nonce_seen_ && nonce == last_stage2_spawn_nonce_) {
            return;
        }
        stage2_spawn_nonce_seen_ = true;
        last_stage2_spawn_nonce_ = nonce;
        const bool rolling_on_enter = (rolling != 0u) || param::config.stage2_rolling_ball_on_enter;
        _randomize_soccer_ball_on_stage2_enter(rolling_on_enter);
    }

    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    using IMUState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::IMUState_>;
    std::unique_ptr<BmsState_t> bmsstate;
    std::unique_ptr<IMUState_t> secondary_imustate;

private:
    static constexpr uint32_t kStage2SpawnMagic = 0x53543245;  // "ST2E"
    uint32_t last_stage2_spawn_nonce_ = 0;
    bool stage2_spawn_nonce_seen_ = false;
};
