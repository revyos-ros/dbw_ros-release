std_msgs/Header header

# Current gear enumeration
Gear gear

# Gear command enumeration
Gear cmd

# Gear command enumeration
Gear driver

# Gear reject enumeration
GearReject reject

#
bool power_latched
bool external_control

# Faults
bool ready # Ready to execute commands
bool override_active  # Active override
bool override_other # Override from other system
bool fault # Fault of any kind
bool bad_crc # CRC error on most recent command

##### Start of slower secondary report #####

# Degradation of any kind
bool degraded

# Command source
CmdSrc cmd_src

# Diagnostics omitted to ruduce message size and clutter
# See GearDiagnostics.msg
