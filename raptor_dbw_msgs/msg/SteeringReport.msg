Header header

# Steering Wheel
float32 steering_wheel_angle      # degrees
float32 steering_wheel_angle_cmd  # degrees
float32 steering_wheel_torque     # 0 to 100%

# Status
bool enabled  
bool driver_activity

bool fault_steering_system

bool overheat_prevention_mode

uint8 rolling_counter

ActuatorControlMode control_type

bool steering_overheat_warning
