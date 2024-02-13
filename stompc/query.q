strategy opt = minE(time) [<=1000]  {x,y} -> {}  : <> DroneAbstract.Target


simulate[<=1000;1]{test} : DroneAbstract.Target under opt
