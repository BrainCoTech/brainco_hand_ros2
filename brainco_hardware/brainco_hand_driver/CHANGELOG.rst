^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package brainco_hand_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2026-01-29)
------------------

* Changed default controller to joint_trajectory_controller
* Changed default configuration to not use namespace
* Added EtherCAT download link in documentation
* Added compilation selection support for different protocols (Modbus/CAN FD/EtherCAT)
* Split driver and moveit packages for better modularity
* Updated SDK to version 1.0.0
* Fixed dlopen error for libusbcanfd.so.1.0.12 library
* Fixed C++ compilation warnings

1.1.0 (2025-11-07)
------------------

* Added CAN FD communication protocol support
* Unified protocol configuration system with YAML-based configs
* ZLG USB-CAN FD driver integration (vendor-embedded)
* Simplified launch parameters with protocol selection
* Enhanced hardware interface for protocol-agnostic operation
* Maintained backward compatibility with existing Modbus setup

1.0.0 (2025-11-06)
------------------

* Initial release of brainco_hand_driver package
* Modbus hardware interface implementation for Revo2 dexterous hands
* Support for both left and right hand configurations
* Integrated ros2_control with trajectory controllers

