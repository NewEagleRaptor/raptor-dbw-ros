# Brake pedal (%)
float32 pedal_cmd

# Enable
bool enable

# Watchdog counter (optional)
uint8 rolling_counter

float32 torque_cmd # %-torque 
float32 accel_limit # m/s^2
float32 decel_limit # m/s^2

ActuatorControlMode control_type