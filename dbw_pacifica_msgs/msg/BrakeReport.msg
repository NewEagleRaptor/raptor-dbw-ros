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

uint8 brake_report_rolling_counter

float32 brake_torque_actual # Nm

# Anti-lock Brakes
bool abs_active
bool abs_enabled

# Stability Control
bool trac_active
bool trac_enabled

# Parking Brake
ParkingBrake parking_brake
