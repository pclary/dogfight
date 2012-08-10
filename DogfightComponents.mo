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
  
    parameter Real x = 0;
    parameter Real y = 0;
  
    RealIO mass;
    VectorIO position;
    VectorIO target;
    Vector velocity;
    input VectorIO thrust;
    
    replaceable Controller controller;
    replaceable Thruster thruster ( mass = 100, T = 2.2, K = 32000, n = 6, maxFuelRate = 7.6 );
    replaceable FuelPump fuelPump ( mass = 2, maxDP = 5, A = 0.08, T = 3 );
    replaceable FuelTank fuelTank ( tare = 10, capacity = 1000 );
    
  initial equation
    position.x = x;
    position.y = y;
    velocity.x = 0;
    velocity.y = 0;
  
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
    connect(fuelPump.fuelRate, fuelTank.fuelRate);
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
    u = Kp * e; // + Ki * I;
    
  end PIController;
  
  
  
  block Controller
    
    input VectorIO position;
    input VectorIO target;
    input RealIO vehicleMass;
    output VectorIO thrust;
    
  protected
    Vector velocity;
    Vector targetVelocity;
    Vector pError;
    Vector vError;
    
    PIController pControl_x ( Kp = 0.05, Ki = 0.003 );
    PIController pControl_y ( Kp = 0.05, Ki = 0.003 );
    PIController vControl_x ( Kp = 0.5, Ki = 0.01 );
    PIController vControl_y ( Kp = 0.5, Ki = 0.01 );
  
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
    vControl_y.e = vError.y + pControl_y.u;
    
    thrust.x = vControl_x.u * vehicleMass;
    thrust.y = vControl_y.u * vehicleMass;
  
  end Controller;
  
  
  
  block Thruster
  
    parameter Real mass = 70;
    parameter Real T = 2;
    parameter Real K = 23000;
    parameter Real n = 4;
    parameter Real maxFuelRate = 3.6;
    
    input VectorIO control;
    input Modelica.Blocks.Interfaces.RealInput thrustDir;
    input RealIO fuelRate;
    output RealIO controlMag;
    output RealIO fuelRateControl;
    output VectorIO thrust;
    output Modelica.Blocks.Interfaces.RealOutput controlDir ( start = 0 );
    
  protected
    Real thrustMag;
    Modelica.Blocks.Continuous.TransferFunction directionLag( b = {1}, a = {T, 1} );
  
  initial equation
    controlDir = directionLag.y;
  
  equation
    controlDir = Modelica.Math.atan3(control.y, control.x, controlDir);
    controlMag = sqrt(control.x^2 + control.y^2);
    thrust.x = thrustMag * cos(thrustDir);
    thrust.y = thrustMag * sin(thrustDir);
    
    fuelRateControl = if -n*log(1 - controlMag/K) < maxFuelRate then -n*log(1 - controlMag/K) else maxFuelRate;
    thrustMag = K*(1 - exp(-fuelRate/n));
    
    connect(controlDir, directionLag.u);
    connect(thrustDir, directionLag.y);
  end Thruster;
  
  
  
  block FuelPump
  
    parameter Real mass = 5;
    parameter Real maxDP = 15;
    parameter Real A = 0.1;
    parameter Real T = 5;
    parameter Real C = 1;
    parameter Real rho = 1000;
    
    input RealIO control;
    input RealIO fuelLevel;
    input Modelica.Blocks.Interfaces.RealInput dP;
    output RealIO fuelRate;
    output Modelica.Blocks.Interfaces.RealOutput controlDP;

  protected
    Modelica.Blocks.Continuous.TransferFunction pumpLag( b = {1}, a = {T, 1} );
    
  equation
    controlDP = if fuelLevel > 0 then if (control / (C*A))^2 / (2*rho) < maxDP then (control / (C*A))^2 / (2*rho) else maxDP else 0;
    fuelRate = C*A*sqrt(2*rho*dP);
    
    connect(controlDP, pumpLag.u);
    connect(dP, pumpLag.y);

  end FuelPump;
  
  
  
  block FuelTank
    
    parameter Real tare = 25;
    parameter Real capacity = 4000;

    input RealIO fuelRate;
    output RealIO fuelLevel;
    output Real mass;

  initial equation
    fuelLevel = capacity;
  
  equation
    mass = tare + fuelLevel;
    fuelRate = -der(fuelLevel);
    
  end FuelTank;
  
  
  
  model Target
  
    parameter Real x = 0;
    parameter Real y = 0;
  
    VectorIO position;
    
  initial equation
    position.x = y;
    position.y = y;
    
  equation
    der(position.x) = sin(time / 100) * 10;
    der(position.y) = cos(time / 100) * 10;
    
  end Target;
  
  
  
end DogfightComponents;

