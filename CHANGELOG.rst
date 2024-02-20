^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dbw_polaris_can
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.8 (2024-02-20)
------------------

2.1.7 (2024-02-12)
------------------

2.1.6 (2024-01-16)
------------------
* Refactor
* Contributors: Kevin Hallenbeck

2.1.5 (2024-01-03)
------------------
* Warn when the incorrect DBW1/DBW2 package is used at runtime and suggest the correct package
* Contributors: Kevin Hallenbeck

2.1.4 (2023-12-13)
------------------
* Remove dataspeed_dbw_gateway package
  The gateway package took a lot of time and memory to compile.
  The gateway package converted messages from specific OEMs to generic messages with the subset of common fields. The DBW2 CAN interface is designed to be the same for all OEMs.
* Contributors: Kevin Hallenbeck

2.1.3 (2023-09-11)
------------------
* Bump firmware versions
* Contributors: Kevin Hallenbeck

2.1.2 (2023-05-10)
------------------
* Bump firmware versions
* Contributors: Kevin Hallenbeck

2.1.1 (2023-01-24)
------------------
* Update install scripts for ROS2
* Contributors: Kevin Hallenbeck

2.1.0 (2022-11-30)
------------------
* Bump firmware versions
* Add missing ament_cmake_gtest dependency
* Sync ament_cmake and ament_cmake_ros in each CMakeLists.txt/package.xml
* Change unsigned vehicle speed to signed vehicle velocity
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

2.0.3 (2022-10-14)
------------------
* Fix socketcan options
* Contributors: Kevin Hallenbeck

2.0.2 (2022-05-13)
------------------
* Periodically publish DBW enabled status in addition to latched and on change
* Bump firmware versions
* Contributors: Kevin Hallenbeck

2.0.1 (2022-02-23)
------------------
* Change parameters to work in Foxy and Galactic
* Export include directories from DBW CAN interface packages
* Contributors: Micho Radovnikovich

2.0.0 (2021-11-03)
------------------
* Initial ROS2 release
* Contributors: Kevin Hallenbeck
