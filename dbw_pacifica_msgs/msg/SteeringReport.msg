Header header

# Steering Wheel
float32 steering_wheel_angle      # degrees
float32 steering_wheel_angle_cmd  # degrees
float32 steering_wheel_torque     # Nm

# Status
bool enabled  # Enabled
bool driver_override

# Watchdog Counter
WatchdogStatus watchdog_status
bool fault_steering_system
bool fault_dbw_system

# Faults
bool fault_bus1
bool fault_bus2
bool fault_calibration
bool fault_power

uint8 steering_report_rolling_counter

SteeringControlMode steering_control_type
