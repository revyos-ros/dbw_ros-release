^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ds_dbw_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.0 (2024-07-17)
------------------
* Add steering offset message
* Add several new messages and signals
  New messages:
  - Battery
  - Low voltage battery state-of-charge/voltage/current/temperature
  - Vehicle ignition
  - BatteryTraction
  - High voltage battery state-of-charge/voltage/temperature
  - DriverAssist
  - ADAS deceleration value
  - FCW/AEB/ACC/BLIS/CTA statuses
  - FuelLevel
  - Fuel level
  - Odometer
  - GPS
  New signals in existing messages:
  - MiscReport
  - Wiper
  - Headlights (high and low beams)
  - Ambient light
  - Outside air temperature
  - ThrottleInfo
  - Drive mode
  - Transmission gear number
* Separate turn signal messages with diagnostics
  Keep functionality in misc cmd/report for a while to ease the transition
* Contributors: Kevin Hallenbeck

2.1.16 (2024-06-17)
-------------------

2.1.15 (2024-06-07)
-------------------
* Brake on comms loss and system lockout
* Contributors: Kevin Hallenbeck

2.1.14 (2024-05-29)
-------------------
* Add gear command option for calibration
* Add gear fault_actuator_config diagnostic
* Contributors: Kevin Hallenbeck

2.1.13 (2024-05-13)
-------------------
* Add gear reject enumeration for excessive vehicle speed
* Add actuator pedal position quality diagnostics
* Add logging status to EcuInfo message
* Contributors: Kevin Hallenbeck

2.1.12 (2024-04-01)
-------------------
* Add support for DBW Monitor module
* Contributors: Kevin Hallenbeck

2.1.11 (2024-03-05)
-------------------
* Add control performance fault
* Add support for system lockout
* Contributors: Kevin Hallenbeck

2.1.10 (2024-02-27)
-------------------

2.1.9 (2024-02-23)
------------------

2.1.8 (2024-02-20)
------------------

2.1.7 (2024-02-12)
------------------
* Add support for system sync option
* Steering wheel angle as degrees instead of radians
* Add brake vacuum pressure
* Contributors: Kevin Hallenbeck

2.1.6 (2024-01-16)
------------------
* Command limits vs vehicle speed
* Report calculated steer value/rate limits
* Report brake/throttle value limits
* Add warning on invalid steer/brake/throttle limit parameters
* Contributors: Kevin Hallenbeck

2.1.5 (2024-01-03)
------------------

2.1.4 (2023-12-13)
------------------
* Single package for all platforms with new DBW2 CAN API
* Contributors: Kevin Hallenbeck
