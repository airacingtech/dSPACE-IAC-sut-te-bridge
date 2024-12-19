#include <chrono>
#include <string>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <vector>
#include <fstream>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"


#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <std_msgs/msg/u_int16.hpp>
#include "rosgraph_msgs/msg/clock.hpp"

#include "autonoma_msgs/msg/ground_truth_array.hpp"
#include "autonoma_msgs/msg/powertrain_data.hpp"
#include "autonoma_msgs/msg/race_control.hpp"
#include "autonoma_msgs/msg/to_raptor.hpp"
#include "autonoma_msgs/msg/vehicle_data.hpp"
#include "autonoma_msgs/msg/vehicle_inputs.hpp"

#include "vectornav_msgs/msg/common_group.hpp"
#include "vectornav_msgs/msg/attitude_group.hpp"
#include "vectornav_msgs/msg/gps_group.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "vectornav_msgs/msg/time_group.hpp"

#include "novatel_oem7_msgs/msg/bestpos.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"
#include "novatel_oem7_msgs/msg/heading2.hpp"
#include "novatel_oem7_msgs/msg/rawimu.hpp"

// Include required msgs for race_common stack
#include "race_msgs/msg/vehicle_control_command.hpp"
#include "race_msgs/msg/steering_report.hpp"
#include "race_msgs/msg/wheel_speed_report.hpp"
#include "race_msgs/msg/engine_report.hpp"
#include "race_msgs/msg/vehicle_command.hpp"
#include "race_msgs/msg/fault_report.hpp"
#include "race_msgs/msg/misc_report.hpp"
#include "race_msgs/msg/vehicle_status.hpp"
#include "race_msgs/msg/tire_pressure_report.hpp"
#include "race_msgs/msg/wheel_potentiometer_report.hpp"
#include "race_msgs/msg/wheel_strain_gauge_report.hpp"
#include "race_msgs/msg/engine_pressures_report.hpp"
#include "race_msgs/msg/tire_temp_report.hpp"
#include "race_msgs/msg/brake_report.hpp"
#include "race_msgs/msg/race_control_rest_of_field.hpp"
#include "race_msgs/msg/vehicle_kinematic_state.hpp"
#include "race_msgs/msg/ride_height_report.hpp"

using RaceControlCommand = race_msgs::msg::VehicleControlCommand;
using RaceSteeringReport = race_msgs::msg::SteeringReport;
using RaceWheelSpeedReport = race_msgs::msg::WheelSpeedReport;
using RaceEngineReport = race_msgs::msg::EngineReport;
using RaceVehicleCommand = race_msgs::msg::VehicleCommand;
using RaceFaultReport = race_msgs::msg::FaultReport;
using RaceMiscReport = race_msgs::msg::MiscReport;
using RaceVehicleStatus = race_msgs::msg::VehicleStatus;
using RaceTirePressureReport = race_msgs::msg::TirePressureReport;
using RaceWheelSpeedReport = race_msgs::msg::WheelSpeedReport;
using RaceWheelPotentiometerReport = race_msgs::msg::WheelPotentiometerReport;
using RaceWheelStrainGaugeReport = race_msgs::msg::WheelStrainGaugeReport;
using RaceEnginePressuresReport = race_msgs::msg::EnginePressuresReport;
using RaceTireTempReport = race_msgs::msg::TireTempReport;
using RaceBrakeReport = race_msgs::msg::BrakeReport;
using RaceRestOfField = race_msgs::msg::RaceControlRestOfField;
using RaceVehicleKinematicState = race_msgs::msg::VehicleKinematicState;
using RaceRideHeightReport = race_msgs::msg::RideHeightReport;

// #include "raptor_dbw_msgs/msg/base_to_car_summary.hpp"
// #include "raptor_dbw_msgs/msg/brake_pressure_report.hpp"

#include "foxglove_msgs/msg/scene_update.hpp"

#include "iac_qos.h"

#include "VESIAPI.h"
#include "VESIResultData.h"

#include "ASMBus_renamed.h"
#include "RaceControlInterface.h"

namespace bridge
{
    class SutTeBridgeNode : public rclcpp::Node
    {

    public:
        SutTeBridgeNode();
        ~SutTeBridgeNode() = default;

    private:
        // Publisher
        rclcpp::Publisher<autonoma_msgs::msg::RaceControl>::SharedPtr raceControlDataPublisher_;
        rclcpp::Publisher<autonoma_msgs::msg::VehicleData>::SharedPtr vehicleDataPublisher_;
        rclcpp::Publisher<autonoma_msgs::msg::PowertrainData>::SharedPtr powertrainDataPublisher_;
        rclcpp::Publisher<autonoma_msgs::msg::GroundTruthArray>::SharedPtr groundTruthArrayPublisher_;

        rclcpp::Publisher<vectornav_msgs::msg::CommonGroup>::SharedPtr verctorNavCommonGroupPublisher_;
        rclcpp::Publisher<vectornav_msgs::msg::AttitudeGroup>::SharedPtr verctorNavAttitudeGroupPublisher_;
        rclcpp::Publisher<vectornav_msgs::msg::ImuGroup>::SharedPtr verctorNavImuGroupPublisher_;
        rclcpp::Publisher<vectornav_msgs::msg::InsGroup>::SharedPtr verctorNavInsGroupPublisher_;
        rclcpp::Publisher<vectornav_msgs::msg::GpsGroup>::SharedPtr verctorNavGpsGroupLeftPublisher_;
        rclcpp::Publisher<vectornav_msgs::msg::GpsGroup>::SharedPtr verctorNavGpsGroupRightPublisher_;
        rclcpp::Publisher<vectornav_msgs::msg::TimeGroup>::SharedPtr verctorNavTimeGroupPublisher_;

        rclcpp::Publisher<vectornav_msgs::msg::GpsGroup>::SharedPtr verctorNavGpsGroupPublisher;

        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr novaTelBestPosPublisher;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr novaTelBestGNSSPosPublisher;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr novaTelBestVelPublisher;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr novaTelBestGNSSVelPublisher;
        rclcpp::Publisher<novatel_oem7_msgs::msg::INSPVA>::SharedPtr novaTelInspvaPublisher;
        rclcpp::Publisher<novatel_oem7_msgs::msg::HEADING2>::SharedPtr novaTelHeading2Publisher;
        rclcpp::Publisher<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr novaTelRawImuPublisher;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr novaTelRawImuXPublisher;

        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr novaTelBestPosPublisher1_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr novaTelBestGNSSPosPublisher1_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr novaTelBestVelPublisher1_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr novaTelBestGNSSVelPublisher1_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::INSPVA>::SharedPtr novaTelInspvaPublisher1_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::HEADING2>::SharedPtr novaTelHeading2Publisher1_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr novaTelRawImuPublisher1_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr novaTelRawImuXPublisher1_;

        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr novaTelBestPosPublisher2_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr novaTelBestGNSSPosPublisher2_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr novaTelBestVelPublisher2_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr novaTelBestGNSSVelPublisher2_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::INSPVA>::SharedPtr novaTelInspvaPublisher2_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::HEADING2>::SharedPtr novaTelHeading2Publisher2_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr novaTelRawImuPublisher2_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr novaTelRawImuXPublisher2_;

        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr foxgloveMapPublisher_;
        rclcpp::Publisher<foxglove_msgs::msg::SceneUpdate>::SharedPtr foxgloveScenePublisher_;
        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr egoPositionPublisher_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr resetCommandPublisher_;

        // Publishers for race_common stack
        rclcpp::Publisher<RaceEngineReport>::SharedPtr pub_engine_report_;
        rclcpp::Publisher<RaceSteeringReport>::SharedPtr pub_steering_report_;
        // rclcpp::Publisher<RaceVehicleCommand>::SharedPtr pub_vehicle_command_;
        // rclcpp::Publisher<RaceVehicleStatus>::SharedPtr pub_vehicle_status_;
        // rclcpp::Publisher<RaceFaultReport>::SharedPtr pub_fault_report_;
        rclcpp::Publisher<RaceMiscReport>::SharedPtr pub_misc_report_;
        rclcpp::Publisher<RaceTirePressureReport>::SharedPtr pub_pressure_fl_report_;
        rclcpp::Publisher<RaceTirePressureReport>::SharedPtr pub_pressure_fr_report_;
        rclcpp::Publisher<RaceTirePressureReport>::SharedPtr pub_pressure_rl_report_;
        rclcpp::Publisher<RaceTirePressureReport>::SharedPtr pub_pressure_rr_report_;
        rclcpp::Publisher<RaceWheelSpeedReport>::SharedPtr pub_wheel_speed_report_;
        rclcpp::Publisher<RaceWheelPotentiometerReport>::SharedPtr pub_wheel_pot_report_;
        rclcpp::Publisher<RaceWheelStrainGaugeReport>::SharedPtr pub_wheel_strain_report_;
        rclcpp::Publisher<RaceEnginePressuresReport>::SharedPtr pub_engine_pressure_report_;
        rclcpp::Publisher<RaceTireTempReport>::SharedPtr pub_tire_temp_report_;
        rclcpp::Publisher<RaceBrakeReport>::SharedPtr pub_brake_report_;
        // rclcpp::Publisher<RaceRestOfField>::SharedPtr pub_rest_of_field_;
        // rclcpp::Publisher<RaceRideHeightReport>::SharedPtr pub_ride_height_;
        // rclcpp::Publisher<RaptorAccCmd>::SharedPtr pub_accel_command_;
        // rclcpp::Publisher<RaptorBrakeCmd>::SharedPtr pub_brake_command_;
        // rclcpp::Publisher<RaptorSteeringCmd>::SharedPtr pub_steer_command_;
        // rclcpp::Publisher<RaptorGearCmd>::SharedPtr pub_gear_command_;
        // rclcpp::Publisher<RaptorDashSwitchCmd>::SharedPtr pub_dash_switch_cmd_;
        // rclcpp::Publisher<RaptorCtVehicleAccFeedback>::SharedPtr pub_ct_vehicle_acc_feedback_;
        // rclcpp::Publisher<raptor_dbw_msgs::msg::BaseToCarSummary>::SharedPtr baseToCarSummaryDataPublisher_;
        // rclcpp::Publisher<raptor_dbw_msgs::msg::BrakePressureReport>::SharedPtr brakePressureReportDataPublisher_;

        // Timer
        rclcpp::TimerBase::SharedPtr updateVESIVehicleInputs_;

        // Subsciber
        // rclcpp::Subscription<autonoma_msgs::msg::VehicleInputs>::SharedPtr receiveVehicleCommands_;
        rclcpp::Subscription<RaceControlCommand>::SharedPtr receiveVehicleCommands_;
        rclcpp::Subscription<autonoma_msgs::msg::ToRaptor>::SharedPtr receiveRaptorCommands_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr useCustomRaceControlSource_;
        rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr simTimeIncrease_;
        rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr simClockTimePublisher_;

        // Parameter
        bool maneuverStarted = false;
        bool vesiDataAvailabe = false;
        bool feedbackDataAvailabe = false;
        bool raptorDataAvailabe = false;
        bool useCustomRaceControl = false;
        bool verbosePrinting = false;
        bool simModeEnabled = false;
        bool numberWarningSent = false;
        uint8_t prestart_rolling_counter;


        // Execution duration logging
        std::vector <double> measured_vesi_times;
        int64_t timeStartNanosec = 0;
        int64_t timeEndNanosec = 0;
        int64_t timeStartVESICallBackNanosec = 0;
        int64_t timeEndVESICallBackNanosec = 0;
        std::ofstream myfile;
        std::string pathTimeRecord;
        bool enableTimeRecord;

        // Simulated clock
        uint32_t nsec = 0;
        uint32_t sec = 0;
        uint64_t simTotalMsec = 0;
        rosgraph_msgs::msg::Clock simClockTime;
        rclcpp::TimerBase::SharedPtr updateSimClock_;

        // Custom publish frequencies
        uint32_t pubIntervalRaceControlData;
        uint32_t pubIntervalVehicleData;
        uint32_t pubIntervalPowertrainData;
        uint32_t pubIntervalGroundTruthArray;
        uint32_t pubIntervalVectorNavData;
        uint32_t pubIntervalNovatelData;
        uint32_t pubIntervalFoxgloveMap;

        // Custom Structures
        VESIResultData feedbackCmds;
        VESIAPI api;
        ASMBus *canBus;
        VESIResultData feedbackCmd;

        void initializeFeedback();

        // Callbacks
        void vesiCallback();
        void simClockTimeCallback();
        void initialSimClockPublish();
        void sendVehicleFeedbackToSimulation();
        // void subscribeVehicleCommandsCallback(const autonoma_msgs::msg::VehicleInputs &msg);
        void subscribeVehicleCommandsCallback(const RaceControlCommand &msg);
        void subscribeRaptorCommandsCallback(const autonoma_msgs::msg::ToRaptor &msg);
        void switchRaceControlSourceCallback(const std_msgs::msg::Bool &msg);
        void simTimeIncreaseCallback(const std_msgs::msg::UInt16 &msg);

        // Publishing functions
        void publishFoxgloveMap();
        void publishFoxgloveSceneUpdate();
        void publishSimulationState();
        void publishRaceControlData();
        void publishVehicleData();
        void publishPowertrainData();
        void publishGroundTruthArray();
        void publishVectorNavData();
        void publishNovatelData(uint8_t novatelID);

    };
}