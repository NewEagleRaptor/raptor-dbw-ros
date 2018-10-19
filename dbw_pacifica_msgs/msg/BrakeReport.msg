Header header

# Brake pedal (%)
float32 pedal_position
float32 pedal_output

# Status
bool enabled  # Enabled
bool driver   # Driver activity

# Watchdog Counter
WatchdogStatus watchdog_status
bool fault_brake_system
bool fault_dbw_system

# Faults
bool fault_ch1
bool fault_ch2

uint8 rolling_counter

float32 brake_torque_actual # 0 to 100%

# Anti-lock Brakes and  Stability Control
bool intervention_active
bool intervention_ready

# Parking Brake
ParkingBrake parking_brake

ActuatorControlMode control_type
