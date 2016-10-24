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

