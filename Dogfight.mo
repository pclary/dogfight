model Dogfight
  import DogfightComponents.*;
  Vehicle target( redeclare EscapeController controller, redeclare Thruster thruster ( mass = 70, T = 2, K = 23000, n = 4, maxFuelRate = 3.6 ), x = 1000, y = 1000 );
  Vehicle vehicle;
  
  Real[2] delta;
  Real error;
  
  annotation(experiment(StartTime = 0.0, StopTime = 1000, Tolerance = 1e-006));
equation
  connect(target.position,vehicle.target);
  connect(vehicle.position, target.target);
  
  delta = target.position - vehicle.position;
  error = sqrt(delta[1]^2 + delta[2]^2);
  
end Dogfight;

