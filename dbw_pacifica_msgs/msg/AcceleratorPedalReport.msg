Header header

# Accelerator pedal
# 0 to 100%
float32 pedal_input
float32 pedal_output

# Status
bool enabled
bool ignore_driver
bool override # Driver override
bool driver   # Driver activity

# Watchdog Counter
WatchdogStatus watchdog_status
bool fault_accel_pedal_system
bool fault_dbw_system

# Faults
bool fault_ch1
bool fault_ch2

uint8 rolling_counter

float32 torque_actual #Nm

ActuatorControlMode control_type