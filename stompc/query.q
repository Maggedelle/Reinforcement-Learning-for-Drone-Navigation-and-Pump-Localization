strategy opt = minE(time) [<=1000]  {x,y,yaw,avg_distance} -> {}  : <> DroneController.target


simulate[<=1000;1]{action} : DroneController.target under opt
