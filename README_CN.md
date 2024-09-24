# 使用激光里程计进行仿真
* 使用rf2o_laser_odometry生成里程计
* 修改rf2o_laser_odometry，增加参数"laser_scan_frame_id",完善得到base_frame至laser_scan_frame的tf变换部分。
* 增加用于发布init_pose话题的py文件，此话题rf2o_laser_odometry需用到。
