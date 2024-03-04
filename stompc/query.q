strategy opt = minE(distance_to_goal + time) [<=1000]  {DroneController.DescisionState} -> {x,y,yaw}  : <> (DroneController.target || time >= 10)

simulate[<=1000;1]{action,x,y, yaw, current_step_length} : (DroneController.target || time >= 10) under opt
