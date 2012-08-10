package DogfightComponents



  connector RealIO = Real u;

  connector VectorIO
    Real x;
    Real y;
  end VectorIO;
  
  
  
  record Vector
    Real x;
    Real y;
  end Vector;
  
  
  
  partial model Component
    Real mass;
  end Component;
  
  
  
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
    mass = controller.mass + thruster.mass + fuelPump.mass + fuelTank.mass;
    
    der(position.x) = velocity.x;
    der(position.y) = velocity.y;
    der(velocity.x) = thrust.x / mass;
    der(velocity.y) = thrust.y / mass;
    
    connect(position, controller.position);
    connect(target, controller.target);
    connect(mass, controller.vehicleMass);
    
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
    extends Component;
    
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
    
    mass = 0;
  
  end Controller;
  
  
  
  class Thruster
    extends Component;
    
    
    
  end Thruster;
  
  
  
end DogfightComponents;

