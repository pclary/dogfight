package DogfightComponents



  connector RealInOut = Real;
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  
  
  
  model Vehicle
  
    parameter Real x = 0;
    parameter Real y = 0;
  
    RealOutput mass;
    RealOutput[2] position;
    RealInOut[2] target;
    Real[2] velocity;
    RealInput[2] thrust;
    
    replaceable Controller controller;
    replaceable Thruster thruster ( mass = 100, T = 2.2, K = 32000, n = 6, maxFuelRate = 7.6 );
    replaceable FuelPump fuelPump ( mass = 2, maxDP = 5, A = 0.08, T = 3 );
    replaceable FuelTank fuelTank ( tare = 10, capacity = 1000 );
    
  initial equation
    position[1] = x;
    position[2] = y;
    velocity[1] = 0;
    velocity[2] = 0;
  
  equation
    mass = thruster.mass + fuelPump.mass + fuelTank.mass;
    
    der(position) = velocity;
    der(velocity) = thrust / mass;
    
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
  
  
  
  block Controller
    
    RealInOut[2] position;
    RealInOut[2] target;
    RealInOut vehicleMass;
    RealOutput[2] thrust;
    
  protected
    Modelica.Blocks.Continuous.LimPID[2] pControl ( k = 0.05, Ti = 0.1, yMax = Modelica.Constants.inf, initType = Modelica.Blocks.Types.InitPID.SteadyState, limitsAtInit = false, controllerType = Modelica.Blocks.Types.SimpleController.P );
    Modelica.Blocks.Continuous.LimPID[2] vControl ( k = 0.1, Ti = 0.1, yMax = Modelica.Constants.inf, initType = Modelica.Blocks.Types.InitPID.SteadyState, limitsAtInit = false, controllerType = Modelica.Blocks.Types.SimpleController.P );
    Modelica.Blocks.Continuous.Der[2] velocity;
    Modelica.Blocks.Continuous.Der[2] targetVelocity;
    Modelica.Blocks.Math.Add[2] controlVelocity;
    Modelica.Blocks.Math.Product[2] accel2thrust; 
    
  equation
    connect(position[:], pControl[:].u_m);
    connect(target[:], pControl[:].u_s);
    connect(position[:], velocity[:].u);
    connect(target[:], targetVelocity[:].u);
    connect(targetVelocity[:].y, controlVelocity[:].u1);
    connect(pControl[:].y, controlVelocity[:].u2);
    connect(velocity[:].y, vControl[:].u_m);
    connect(controlVelocity[:].y, vControl[:].u_s);
    connect(vControl[:].y, accel2thrust[:].u1);
    connect(vehicleMass, accel2thrust[1].u2);
    connect(vehicleMass, accel2thrust[2].u2);
    connect(accel2thrust[:].y, thrust[:]);
  
  end Controller;
  
  
  
  block Thruster
  
    parameter Real mass = 70;
    parameter Real T = 2;
    parameter Real K = 23000;
    parameter Real n = 4;
    parameter Real maxFuelRate = 3.6;
    
    RealInput[2] control;
    input Modelica.Blocks.Interfaces.RealInput thrustDir;
    RealInput fuelRate;
    RealOutput controlMag;
    RealOutput fuelRateControl;
    RealOutput[2] thrust;
    output Modelica.Blocks.Interfaces.RealOutput controlDir ( start = 0 );
    
  protected
    Real thrustMag;
    Modelica.Blocks.Continuous.TransferFunction directionLag( b = {1}, a = {T, 1} );
  
  initial equation
    controlDir = directionLag.y;
  
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
    RealInput fuelLevel;
    input Modelica.Blocks.Interfaces.RealInput dP;
    RealOutput fuelRate;
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

    RealInput fuelRate;
    RealOutput fuelLevel;
    output Real mass;

  initial equation
    fuelLevel = capacity;
  
  equation
    mass = tare + fuelLevel;
    fuelRate = -der(fuelLevel);
    
    when fuelLevel < 0 then
      reinit(fuelLevel, 0);
    end when;
    
  end FuelTank;
  
  
  
  model Target
  
    parameter Real x = 0;
    parameter Real y = 0;
  
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

