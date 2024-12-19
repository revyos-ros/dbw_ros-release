std_msgs/Header header

# Turn Signal enumeration
TurnSignal turn_signal

# Parking brake
PrkBrkStat parking_brake

# Passenger seat
bool passenger_detect
bool passenger_airbag

# Seat Belts
bool buckle_driver
bool buckle_passenger

# Doors
bool door_driver
bool door_passenger
bool door_rear_left
bool door_rear_right
bool door_hood
bool door_trunk

# Buttons
bool btn_ld_ok        # Left D-Pad OK
bool btn_ld_up        # Left D-Pad Up
bool btn_ld_down      # Left D-Pad Down
bool btn_ld_left      # Left D-Pad Left
bool btn_ld_right     # Left D-Pad Right
bool btn_rd_ok        # Right D-Pad OK
bool btn_rd_up        # Right D-Pad Up
bool btn_rd_down      # Right D-Pad Down
bool btn_rd_left      # Right D-Pad Left
bool btn_rd_right     # Right D-Pad Right
bool btn_cc_mode      # Cruise Control Mode
bool btn_cc_on        # Cruise Control On
bool btn_cc_off       # Cruise Control Off
bool btn_cc_res       # Cruise Control Resume
bool btn_cc_cncl      # Cruise Control Cancel
bool btn_cc_on_off    # Cruise Control On/Off Toggle
bool btn_cc_res_cncl  # Cruise Control Resume/Cancel
bool btn_cc_res_inc   # Cruise Control Resume+
bool btn_cc_res_dec   # Cruise Control Resume-
bool btn_cc_set_inc   # Cruise Control Set+
bool btn_cc_set_dec   # Cruise Control Set-
bool btn_acc_gap_inc  # Adaptive Cruise Control Gap+
bool btn_acc_gap_dec  # Adaptive Cruise Control Gap-
bool btn_limit_on_off # Speed Limiter On/Off Toggle
bool btn_la_on_off    # Lane Assist On/Off Toggle
bool btn_apa          # Active Park Assist
bool btn_media        # Media (source)
bool btn_vol_inc      # Volume increment
bool btn_vol_dec      # Volume decrement
bool btn_mute         # Volume mute
bool btn_speak        # Speak
bool btn_prev         # Media previous
bool btn_next         # Media next
bool btn_call_start   # Call start/answer
bool btn_call_end     # Call end/disconnect

# Front Windshield Wipers enumeration
Wiper wiper

# Headlight enumerations
bool headlight_low
bool headlight_high
HeadlightCtrlLow headlight_low_control
HeadlightCtrlHigh headlight_high_control

# Ambient Light Sensor enumeration
AmbientLight ambient_light

# Outside air temperature
float32 outside_air_temp # Degrees Celsius
