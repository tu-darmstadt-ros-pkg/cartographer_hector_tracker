screen -S "bag_processing" -d -m
screen -r "bag_processing" -X stuff $'roslaunch cartographer_hector_tracker simulation_small_loop_processing.launch\n'
sleep 5s #Odometry filtering requires a few seconds to deliver consistent results
screen -S "cartographer" -d -m
screen -r "cartographer" -X stuff $'roslaunch cartographer_hector_tracker simulation_small_loop_cartographer_vlp16_pg.launch\n'
