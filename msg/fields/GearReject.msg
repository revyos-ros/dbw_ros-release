uint8 value

uint8 NONE=0              # Not rejected
uint8 FAULT=1             # System in fault state
uint8 UNSUPPORTED=2       # Unsupported gear command
uint8 SHIFT_IN_PROGRESS=3 # Shift in progress
uint8 OVERRIDE=4          # Override on brake, throttle, or steering
uint8 BRAKE_HOLD=5        # Brake hold time depleted, stay in park
uint8 VEHICLE=7           # Rejected by vehicle (try pressing the brakes)
