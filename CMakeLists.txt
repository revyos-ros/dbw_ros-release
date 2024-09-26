cmake_minimum_required(VERSION 3.5)
project(ds_dbw_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/fields/AmbientLight.msg"
  "msg/fields/CmdSrc.msg"
  "msg/fields/DecelSrc.msg"
  "msg/fields/DriveMode.msg"
  "msg/fields/Gear.msg"
  "msg/fields/GearNum.msg"
  "msg/fields/GearReject.msg"
  "msg/fields/HeadlightCtrlHigh.msg"
  "msg/fields/HeadlightCtrlLow.msg"
  "msg/fields/Ignition.msg"
  "msg/fields/OnePedal.msg"
  "msg/fields/PrkBrkCmd.msg"
  "msg/fields/PrkBrkStat.msg"
  "msg/fields/Quality.msg"
  "msg/fields/SystemState.msg"
  "msg/fields/SystemSyncMode.msg"
  "msg/fields/TurnSignal.msg"
  "msg/fields/Wiper.msg"
  "msg/Battery.msg"
  "msg/BatteryTraction.msg"
  "msg/BrakeCmd.msg"
  "msg/BrakeDiagnostics.msg"
  "msg/BrakeInfo.msg"
  "msg/BrakeReport.msg"
  "msg/DriverAssist.msg"
  "msg/EcuInfo.msg"
  "msg/FuelLevel.msg"
  "msg/GearCmd.msg"
  "msg/GearDiagnostics.msg"
  "msg/GearReport.msg"
  "msg/MiscCmd.msg"
  "msg/MiscReport.msg"
  "msg/MonitorCmd.msg"
  "msg/MonitorReport.msg"
  "msg/MonitorThrottle.msg"
  "msg/SteeringCmd.msg"
  "msg/SteeringDiagnostics.msg"
  "msg/SteeringOffset.msg"
  "msg/SteeringReport.msg"
  "msg/SystemReport.msg"
  "msg/ThrottleCmd.msg"
  "msg/ThrottleDiagnostics.msg"
  "msg/ThrottleInfo.msg"
  "msg/ThrottleReport.msg"
  "msg/TirePressures.msg"
  "msg/TurnSignalCmd.msg"
  "msg/TurnSignalReport.msg"
  "msg/UlcCmd.msg"
  "msg/UlcReport.msg"
  "msg/VehicleVelocity.msg"
  "msg/WheelPositions.msg"
  "msg/WheelSpeeds.msg"
  DEPENDENCIES
  "std_msgs"
  "geometry_msgs"
)

ament_package()
