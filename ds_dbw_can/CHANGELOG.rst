^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ds_dbw_can
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.10 (2024-02-27)
-------------------

2.1.9 (2024-02-23)
------------------
* Add missing rclcpp_components dependency to package.xml
* Contributors: Kevin Hallenbeck

2.1.8 (2024-02-20)
------------------
* Bump firmware versions to match 2024/02/21 release package
* PlatformMap as sparse std::map instead of dense std::array
* Platform/Module from EcuInfo
* Rename LimitHash to ParamHash
* Fix ULC config message transmit rate
* Add warnings for ULC preemption and lack of CRC/RC validation
* ULC demo scripts converted to DBW 2
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

2.1.7 (2024-02-12)
------------------
* Bump firmware versions to match 2024/02/12 release package
* Add support for system sync option
* Fix unused brake accel command scaling
* Steering wheel angle as degrees instead of radians
* Add brake vacuum pressure
* Contributors: Kevin Hallenbeck

2.1.6 (2024-01-16)
------------------
* Command limits vs vehicle speed
* Report calculated steer value/rate limits
* Report brake/throttle value limits
* Add warning on invalid steer/brake/throttle limit parameters
* Print limit hashes
* Refactor
* Add reserved CAN message
* Contributors: Kevin Hallenbeck

2.1.5 (2024-01-03)
------------------
* Bump firmware versions to match 2024/01/02 release package
* Warn when the incorrect DBW1/DBW2 package is used at runtime and suggest the correct package
* Contributors: Kevin Hallenbeck

2.1.4 (2023-12-13)
------------------
* Single package for all platforms with new DBW2 CAN API
* Contributors: Kevin Hallenbeck
