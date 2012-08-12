model Dogfight
  import DogfightComponents.*;
  Target target( x = 1000, y = 1000);
  Vehicle vehicle;
  annotation(experiment(StartTime = 0.0, StopTime = 1000, Tolerance = 1e-006));
equation
  connect(target.position,vehicle.target);
  connect(vehicle.position, target.target);
end Dogfight;

