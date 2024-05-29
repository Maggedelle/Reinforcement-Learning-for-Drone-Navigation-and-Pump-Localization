strategy opt = maxE(accum_reward - time) [<=21]{DroneController.DescisionState}->{yaw,x,y} : <> (DroneController.target || time >= 20)
 
simulate [<=1000;1] {action} : (DroneController.target || time >= 10) under opt