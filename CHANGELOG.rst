^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rs_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
v2.2.0 (2019-3-4)
---
New features:
新特性:
- 改进四元数积分公式
- 可配置地图显示的点云数量比例
- 修复有时程序无法正常退出的问题
- 增加单独发布点云对应的定位结果

v2.1.0 (2018-12-3)
---
New features:
* 3D Kinematic model
* Support initialization with RTK
* Sensor data communication monitoring
* Support rsmap V0.0.3
* Lidar pose w.r.t car frame is configurabl

2.0.3 (2018-11-12)
-------------------
* publish status of initialization of localization for testing requirements.

2.0.2 (2018-10-26)
-------------------
* new configuration procedure, use user_config/user_config.yaml

2.0.1 (2018-10-18)
-------------------
* Implement GNSS observer (disabled by default) need driver support for cov
* fix odom cov too large
* update point_cloud.cpp to the latest
* Fix quaternion equation in EKF
* code optimization and reorganize
* use different value for icp heading dir noise and side dir noisey
* Contributor: yczhang

2.0.0 (2018-09-27)
-------------------
* new msf framework
* Contributor: Yufan