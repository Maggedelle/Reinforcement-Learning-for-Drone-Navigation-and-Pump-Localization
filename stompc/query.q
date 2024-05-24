strategy opt = maxE(accum_reward - time) [<=2]{DroneController.DescisionState}->{yaw,x,y} : <> (DroneController.target || time >= 1)
 
simulate [<=1000;1] {action} : (DroneController.target || time >= 10) under opt