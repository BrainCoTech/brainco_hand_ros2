^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package revo2_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

* Add per-joint integral velocity control with configurable gain and contribution
  limit, conditional anti-windup and lifecycle/timeout resets (2026-11-18).
  Integral gain defaults to zero to preserve existing PD behavior.

* Add B-compatible joint trajectory controllers and selectable startup modes,
  including independent mode overrides for dual hands (2026-11-13).

* Add explicit SDK file logging and manual touch enable/calibration ROS services
  with raw transport status, mask validation and serialized device access (2026-11-13).

* Upgrade the vendored Stark SDK and downloader to v2.0.9 (2026-11-12).
* Decode device information with the matching v2.0.9 public ABI instead of the
  v1.5.1 legacy layout workaround.
* Validate SDK downloads and platform caches, and install the SDK license.

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

* Initial release of revo2_driver package
* Modbus hardware interface implementation for Revo2 dexterous hands
* Support for both left and right hand configurations
* Integrated ros2_control with trajectory controllers

