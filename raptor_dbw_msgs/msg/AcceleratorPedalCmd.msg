# Accelerator pedal (%)
float32 pedal_cmd

float32 torque_cmd # %-torque

float32 speed_cmd # m/s

# Enable
bool enable

# Ignore driver overrides
bool ignore

# Watchdog counter (optional)
uint8 rolling_counter

float32 road_slope # degrees - used with speed mode

ActuatorControlMode control_type

float32 accel_limit # m/s^2

float32 accel_positive_jerk_limit # m/s^3