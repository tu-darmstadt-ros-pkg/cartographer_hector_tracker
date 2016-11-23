# cartographer_hector_tracker
Provides Taurob Tracker (Team Hector variant) integration for Cartographer.


# Simulation Dataset

A simulation dataset of a Tracker robot equipped with a spinning Hokuyo UTM-30LX LIDAR can be run as described in the following. Odometry for this dataset equals ground truth, so good results are be expected and this mainly serves to verify that cartographer does nothing stupid with very good input data. The recording of the dataset can be seen in this video:

[![IMAGE ALT TEXT](http://img.youtube.com/vi/KIdIpFkSiHE/0.jpg)](https://youtu.be/KIdIpFkSiHE "Video Title")

To run cartographer on the corresponding bag file:
* Download [2016-08-04-15-17-14.bag](https://drive.google.com/open?id=0B1hU91jkd7VwbU5JZ3hqenNiT2c)
* Run the associated demo launch file:
```
roslaunch cartographer_hector_tracker demo_hector_tracker_spin_lidar.launch bag_filename:=${HOME}/Downloads/2016-08-04-15-17-14.bag
```

# Real Robot Dataset

A dataset recorded at one of our labs at TU Darmstadt with a real Tracker robot can also be used. It can be downloaded here: [tracker_tuda_site_loop.bag](https://drive.google.com/open?id=0B2xI6QMWN-f6TE1Zd3hHbVVvR1E). The bag file contains the following data relevant for mapping using cartographer:
* `/imu/data` IMU data of a UM7 IMU mounted in the robot base (data frame_id: base_link)
* `/odom` Fused Odometry data using IMU data for orientation and track/drivetrain odometry velocity for velocity along the x axis
* `/spin_laser/scan` Raw spinning LIDAR scans
* `/scan_cloud_filtered` Filtered spinning LIDAR scan data as a point cloud. The data is self filtered and filtered for scan shadow and low intensity points.
* `/tf` Tf data for the whole robot including odom to base_link, but missing the map to odom transform which is to be estimated by SLAM.
