package DogfightComponents

  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  
  
  
  model Vehicle
  
    parameter Real x = 0;
    parameter Real y = 0;
  
    RealOutput mass;
    RealOutput[2] position;
    RealInput[2] target;
    Real[2] velocity;
    RealInput[2] thrust;
    
    replaceable Controller controller;
    replaceable Thruster thruster ( mass = 100, T = 2.2, K = 32000, n = 6, maxFuelRate = 7.6 );
    replaceable FuelPump fuelPump ( mass = 2, maxDP = 5, A = 0.08, T = 3 );
    replaceable FuelTank fuelTank ( tare = 10, capacity = 1000 );
    
  initial equation
    position = {x, y};
    velocity = {0, 0};
  
  equation
    mass = thruster.mass + fuelPump.mass + fuelTank.mass;
    
    der(position) = velocity;
    der(velocity) = thrust / mass;
    
    connect(position, controller.position);
    connect(target, controller.target);
    connect(mass, controller.vehicleMass);
    connect(controller.thrust, thruster.control);
    connect(thruster.fuelRateControl, fuelPump.control);
    connect(fuelPump.tankControl, fuelTank.control);
    connect(fuelTank.fuelRate, fuelPump.fuelRateIn);
    connect(fuelPump.fuelRateOut, thruster.fuelRate);
    connect(thruster.thrust, thrust);
    
  end Vehicle;
  
  
  
  block PIController
  
    parameter Real Kp = 1;
    parameter Real Ki = 1;
    parameter Real ILim = Modelica.Constants.inf;
    RealInput u_s;
    RealInput u_m;
    RealOutput y;
    
  protected
    Real e;
    Real I;
  
  initial equation
    I = 0;
  
  equation
    e = u_s - u_m;
    der(I) = if I < -ILim and e < 0 or I > ILim and e > 0 then 0 else e;
    y = Kp * e + Ki * I;
    
  end PIController;

  
  
  block Controller
    
    parameter Real Kp_p = 0.09;
    parameter Real Ki_p = 0;
    parameter Real Kp_v = 0.18;
    parameter Real Ki_v = 0.02;
    
    RealInput[2] position;
    RealInput[2] target;
    RealInput vehicleMass;
    RealOutput[2] thrust;
    
  protected
    PIController[2] pControl ( Kp = Kp_p, Ki = Ki_p );
    PIController[2] vControl ( Kp = Kp_v, Ki = Ki_v, ILim = 80 );
    Modelica.Blocks.Continuous.Der[2] velocity;
    Modelica.Blocks.Continuous.Der[2] targetVelocity;
    Modelica.Blocks.Math.Add[2] controlVelocity;
    Modelica.Blocks.Math.Product[2] accel2thrust; 
  
  initial equation
    thrust = {0, 0};
  
  equation
    connect(position, pControl.u_m);
    connect(target, pControl.u_s);
    connect(position, velocity.u);
    connect(target, targetVelocity.u);
    connect(targetVelocity.y, controlVelocity.u1);
    connect(pControl.y, controlVelocity.u2);
    connect(velocity.y, vControl.u_m);
    connect(controlVelocity.y, vControl.u_s);
    connect(vControl.y, accel2thrust.u1);
    connect(vehicleMass, accel2thrust[1].u2);
    connect(vehicleMass, accel2thrust[2].u2);
    thrust = if time > 0 then accel2thrust.y else {0, 0};
  
  end Controller;
  
  
  
  block EscapeController
    
    parameter Real maxThrust = 32000;
    
    RealInput[2] position;
    RealInput[2] target;
    RealInput vehicleMass;
    RealOutput[2] thrust;
    
  protected
    Real direction;
  
  initial equation
    thrust = {0, 0};
  
  equation
    direction = atan2(position[2] - target[2], position[1] - target[1]);
    thrust[1] = if time > 0 then cos(direction) * maxThrust else 0;
    thrust[2] = if time > 0 then sin(direction) * maxThrust else 0;
  
  end EscapeController;
  
  
  
  block DebugController
    
    RealInput[2] position;
    RealInput[2] target;
    RealInput vehicleMass;
    RealOutput[2] thrust;
    
  initial equation
    thrust = {0, 0};
    
  equation
    thrust = target - position;
    //thrust = if time > 0 then target - position else {0, 0};
  
  end DebugController;
  
  
  
  block Thruster
  
    parameter Real mass = 70;
    parameter Real T = 2;
    parameter Real K = 23000;
    parameter Real n = 4;
    parameter Real maxFuelRate = 3.6;
    
    RealInput[2] control;
    RealInput thrustDir;
    RealInput fuelRate;
    RealOutput controlMag;
    RealOutput fuelRateControl;
    RealOutput[2] thrust;
    RealOutput controlDir;
    
  protected
    Real thrustMag;
    Modelica.Blocks.Continuous.TransferFunction directionLag( b = {1}, a = {T, 1} );
  
  initial equation
    thrustMag = 0;
    thrustDir = 0;
  
  equation
    controlDir = Modelica.Math.atan3(control[2], control[1], controlDir);
    controlMag = sqrt(control[1]^2 + control[2]^2);
    thrust[1] = thrustMag * cos(thrustDir);
    thrust[2] = thrustMag * sin(thrustDir);
    
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
    
    RealInput control;
    RealInput fuelRateIn;
    RealOutput tankControl;
    RealOutput fuelRateOut;

  protected
    Real dP;
    Modelica.Blocks.Continuous.TransferFunction pumpLag( b = {1}, a = {T, 1} );
    
  initial equation
    dP = 0;
    fuelRateIn = 0;
    fuelRateOut = 0;
    
  equation
    dP = (control / (C*A))^2 / (2*rho);
    tankControl = if dP < maxDP then C*A*sqrt(2*rho*dP) else C*A*sqrt(2*rho*maxDP);
    
    connect(fuelRateIn, pumpLag.u);
    connect(pumpLag.y, fuelRateOut);

  end FuelPump;
  
  
  
  block FuelTank
    
    parameter Real tare = 25;
    parameter Real capacity = 4000;

    RealInput control;
    RealOutput fuelRate;
    RealOutput mass;
    
  protected
    Real fuelLevel;

  initial equation
    fuelLevel = capacity;
    fuelRate = 0;
  
  equation
    fuelRate = if fuelLevel > 0 then control else 0;
    mass = tare + fuelLevel;
    fuelRate = -der(fuelLevel);

  end FuelTank;
  
  
  
  model Target
  
    parameter Real x = 0;
    parameter Real y = 0;
    
    RealInput[2] target;
    RealOutput[2] position;
    Real[2] velocity;
    
  initial equation
    position[1] = x;
    position[2] = y;
    
  equation
    velocity = der(position);
    velocity[1] = sin(time / 20) * 10;
    velocity[2] = cos(time / 20) * 10;
    
  end Target;
  
  
  
end DogfightComponents;

