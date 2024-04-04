strategy opt = maxE(accum_reward) [<=1000]{DroneController.DescisionState,x,y}->{yaw} : <> (DroneController.target || time >= 10)
 
simulate [<=1000;1] {action} : (DroneController.target || time >= 20) under opt