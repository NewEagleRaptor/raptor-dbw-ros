# Steering Wheel
float32 steering_wheel_angle_cmd        # degrees, range -500 to 500
float32 steering_wheel_angle_velocity   # degrees/s, range 0 to 498, 0 = maximum

# Enable
bool enable

# Clear driver overrides
bool clear

# Ignore driver overrides
bool ignore

# Disable the driver override audible warning
bool quiet

# Watchdog counter (optional)
uint8 count

SteeringCommandType command_type

float32 steering_wheel_torque_cmd

ControlMode control_mode
