package DogfightComponents



  connector RealIO = Real;

  connector VectorIO
    Real x;
    Real y;
  end VectorIO;
  
  
  
  record Vector
    Real x;
    Real y;
  end Vector;
  
  
  
  model Vehicle
  
    RealIO mass;
    VectorIO position;
    VectorIO target;
    Vector velocity;
    input VectorIO thrust;
    
    replaceable Controller controller;
    replaceable Thruster thruster;
    replaceable FuelPump fuelPump;
    replaceable FuelTank fuelTank;
    
  equation
    mass = thruster.mass + fuelPump.mass + fuelTank.mass;
    
    der(position.x) = velocity.x;
    der(position.y) = velocity.y;
    der(velocity.x) = thrust.x / mass;
    der(velocity.y) = thrust.y / mass;
    
    connect(position, controller.position);
    connect(target, controller.target);
    connect(mass, controller.vehicleMass);
    connect(controller.thrust, thruster.control);
    connect(thruster.fuelRateControl, fuelPump.control);
    connect(fuelPump.fuelLevel, fuelTank.fuelLevel);
    connect(fuelPump.fuelRate, thruster.fuelRate);
    connect(thruster.thrust, thrust);
    
  end Vehicle;
  
  

  block PIController
  
    parameter Real Kp = 1;
    parameter Real Ki = 1;
    input Real e;
    output Real u;
    
  protected
    Real I;
    
  equation
    e = der(I);
    u = Kp * e + Ki * I;
    
  end PIController;
  
  
  
  model Controller
    
    input VectorIO position;
    input VectorIO target;
    input RealIO vehicleMass;
    output VectorIO thrust;
    
  protected
    Vector velocity;
    Vector targetVelocity;
    Vector pError;
    Vector vError;
    
    PIController pControl_x;
    PIController pControl_y;
    PIController vControl_x;
    PIController vControl_y;
  
  equation
    velocity.x = der(position.x);
    velocity.y = der(position.y);
    targetVelocity.x = der(target.x);
    targetVelocity.y = der(target.y);
    
    pError.x = target.x - position.x;
    pError.y = target.y - position.y;
    vError.x = targetVelocity.x - velocity.x;
    vError.y = targetVelocity.y - velocity.y;
    
    pControl_x.e = pError.x;
    pControl_y.e = pError.y;
    vControl_x.e = vError.x + pControl_x.u;
    vControl_y.e = vError.y + pControl_x.u;
    
    thrust.x = vControl_x.u * vehicleMass;
    thrust.y = vControl_y.u * vehicleMass;
  
  end Controller;
  
  
  
  model Thruster
  
    parameter Real mass = 70;
    parameter Real T = 2;
    parameter Real K = 23000;
    parameter Real n = 4;
    parameter Real maxFuelRate = 3.6;
    
    
    input VectorIO control;
    output VectorIO thrust;
    Modelica.Blocks.Interfaces.RealOutput controlDir;
    Modelica.Blocks.Interfaces.RealInput thrustDir;
    output RealIO controlMag;
    Real thrustMag;
    output RealIO fuelRateControl;
    input RealIO fuelRate;
    
    Modelica.Blocks.Continuous.TransferFunction directionLag( b = {1}, a = {T, 1} );
    
  equation
    controlDir = atan2(control.y, control.x);
    controlMag = sqrt(control.x^2 + control.y^2);
    thrust.x = thrustMag * cos(thrustDir);
    thrust.y = thrustMag * sin(thrustDir);
    
    fuelRateControl = if -n*log(1 - controlMag/k) < maxFuelRate then -n*log(1 - controlMag/k) else maxFuelRate;
    thrustMag = k*(1 - exp(-fuelRate/n));
    
    connect(controlDir, directionLag.u);
    connect(thrustDir, directionLag.y);
  end Thruster;
  
  
  
  model FuelPump
  
    parameter Real mass = 5;
    parameter Real maxDP = 15;
    parameter Real A = 0.1;
    parameter Real T = 5;
    parameter Real C = 1;
    parameter Real rho = 1000;
    
    input RealIO control;
    output RealIO fuelRate;
    Modelica.Blocks.Interfaces.RealOutput controlDP;
    Modelica.Blocks.Interfaces.RealInput dP;
    input RealIO fuelLevel;
    
    Modelica.Blocks.Continuous.TransferFunction pumpLag( b = {1}, a = {T, 1} );
    
  equation
    controlDP = if fuelLevel > 0 then if (control / (C*A))^2 / (2*rho) < maxDP then (control / (C*A))^2 / (2*rho) else maxDP else 0;
    fuelRate = C*A*sqrt(2*rho*dP);
    
    connect(controlDP, pumpLag.u);
    connect(dP, pumpLag.y);

  end FuelPump;
  
  
  
  model FuelTank
    
    parameter Real tare = 25;
    parameter Real capacity = 4000;
    
    Real mass;
    RealIO fuelLevel;
    input RealIO fuelRate;
    
  initial equation
    fuelLevel = capacity;
  
  equation
    mass = tare + fuelLevel;
    fuelRate = -der(fuelLevel);
    
  end FuelTank;
  
  
  
end DogfightComponents;

