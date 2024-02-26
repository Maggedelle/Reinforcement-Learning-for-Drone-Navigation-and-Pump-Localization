strategy opt = minE(time) [<=1000]  {x,y,yaw,avg_distance, drone_state} -> {}  : <> DroneController.target


simulate[<=1000;1]{action,x,y, yaw, current_step_length} : DroneController.target under opt
