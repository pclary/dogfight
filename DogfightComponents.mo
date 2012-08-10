package DogfightComponents
  
  import Modelica.Blocks.Interfaces.*;

  class Vector
    Real x;
    Real y;
    Real direction;
    Real magnitude;
  equation
    magnitude = sqrt(x ^ 2 + y ^ 2);
    direction = atan2(y, x);
  end Vector;
  
  partial model Component
    Real mass;
  end Component;
  
  model Vehicle
  
    RealOutput position_x;
    RealOutput position_y;
    Real velocity_x;
    Real velocity_y;
    
    RealInput target_x;
    RealInput target_y;
    
    RealInput thrust_x;
    RealInput thrust_y;
    
    Real mass;
    
    replaceable Controller controller;
    replaceable Thruster thruster;
    replaceable FuelPump fuelPump;
    replaceable FuelTank fuelTank;
    
  equation
  
    mass = controller.mass + thruster.mass + fuelPump.mass + fuelTank.mass;
    
    der(position_x) = velocity_x;
    der(position_y) = velocity_y;
    der(velocity_x) = thrust_x / mass;
    der(velocity_y) = thrust_y / mass;
    
  end Vehicle;
  
end DogfightComponents;

