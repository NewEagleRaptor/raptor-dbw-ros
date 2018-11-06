# Steering Wheel
float32 angle_cmd        # degrees, range -500 to 500
float32 angle_velocity   # degrees/s, range 0 to 498, 0 = maximum

# Enable
bool enable

# Ignore driver overrides
bool ignore

# Watchdog counter (optional)
uint8 rolling_counter

float32 torque_cmd # %-torque

float32 vehicle_curvature_cmd # 1/m

ActuatorControlMode control_type