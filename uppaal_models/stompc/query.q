strategy opt = minE(time) [<=1000]  {x,y} -> {}  : <> DroneAbstract.Target


simulate[<=1000;1]{x,y} : DroneAbstract.Target under opt
