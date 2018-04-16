uint8 source

uint8 NONE=0                        # No source for watchdog counter fault
uint8 OTHER_BRAKE=1                 # Fault determined by brake controller
uint8 OTHER_ACCELERATOR_PEDAL=2     # Fault determined by accelerator pedal controller
uint8 OTHER_STEERING=3              # Fault determined by steering controller
uint8 BRAKE_COUNTER=4               # Brake command counter failed to increment
uint8 BRAKE_DISABLED=5              # Brake transition to disabled while in gear or moving
uint8 BRAKE_COMMAND=6               # Brake command timeout after 100ms
uint8 BRAKE_REPORT=7                # Brake report timeout after 100ms
uint8 ACCELERATOR_PEDAL_COUNTER=8   # Accelerator pedal command counter failed to increment
uint8 ACCELERATOR_PEDAL_DISABLED=9  # Accelerator pedal transition to disabled while in gear or moving
uint8 ACCELERATOR_PEDAL_COMMAND=10  # Accelerator pedal command timeout after 100ms
uint8 ACCELERATOR_PEDAL_REPORT=11   # Accelerator pedal report timeout after 100ms
uint8 STEERING_COUNTER=12           # Steering command counter failed to increment
uint8 STEERING_DISABLED=13          # Steering transition to disabled while in gear or moving
uint8 STEERING_COMMAND=14           # Steering command timeout after 100ms
uint8 STEERING_REPORT=15            # Steering report timeout after 100ms
