function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)

    % aircraft_state = [xi, yi, zi, roll, pitch, yaw, u, v, w, p, q, r]
    % extract states
    x = aircraft_state(1,1);
    y = aircraft_state(2,1);
    z = aircraft_state(3,1);
    phi = aircraft_state(4,1); 
    theta = aircraft_state(5,1);
    psi = aircraft_state(6,1);
    u = aircraft_state(7,1);
    v = aircraft_state(8,1);
    w = aircraft_state(9,1);
    p = aicraft_state(10,1);
    q = aircraft_state(11,1);
    r = aircraft_state(12,1); 
    
    % aircraft_surfaces = [de da dr dt];
    % extract control surfaces
    de = aircraft_surfaces(1,1); 
    da = aircraft_surfaces(2,1);
    dr = aircraft_surfaces(3,1);
    dt = aircraft_surfaces(4,1);
    
    % density = aircraft_parameters.rho; ??
    g = aircraft_parameters.g; 
    m = aircraft_paramters.m;
    
    % calculate aero forces and moments
    [aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);
    
    % extract aero forces
    X = aero_forces(1); 
    Y = aero_forces(2);
    Z = aero_forces(3);
    
    % extract aero moments
    L = aero_moments(1);
    M = aero_moments(2);
    N = aero_moments(3);
    
    % extract moments of inertia
    Ix = aircraft_parameters.Ix; 
    Iy = aircraft_parameters.Iy;
    Iz = aircraft_parameters.Iz;
    Ixz = aircraft_parameters.Ixz; 

    % initialize xdot vector
    xdot = zeros(12,1); 
    
    %% Kinematic Equations: 
    pE = TransformFromInertialToBody(I, aircraft_state(7:9,1)); 
    % extract x_dot_E 
    % xdot(1) = ((cos(theta)*cos(psi))*u) + (((sin(phi)*sin(theta)*cos(psi))- (cos(phi)*sin(psi)))*v) + (((cos(phi)*sin(theta)*cos(psi))+(sin(phi)*sin(psi)))*w); 
    xdot(1) = pE(1,1); 

    % solve for y_dot_E 
    % xdot(2) = ((cos(theta)*sin(psi))*u) + (((sin(phi)*sin(theta)*sin(psi)) + (cos(phi)*cos(psi)))*v) + (((cos(phi)*sin(theta)*sin(psi)) - (sin(phi)*cos(psi)))*w); 
    xdot(2) = pE(2,1);

    % solve for z_dot_E 
    % xdot(3) = ((-sin(theta))*u) + ((sin(phi)*cos(theta))*v) + ((cos(phi)*cos(theta))*w); 
    xdot(3) = pE(3,1);
    
    %% Euler angle derivatives
    % solve for phi dot
    xdot(4) = p + (q * sin(phi) * tan(theta)) + (r * cos(phi) * tan(theta)); 

    % solve for theta dot
    xdot(5) = (q * cos(phi)) - (r * sin(phi)); 

    % solve for psi dot
    xdot(6) = (q * sin(phi) * sec(theta)) + (r * cos(phi) * sec(theta)); 
    
    %% Translational derivatives
    % solve for u dot
    xdot(7) = r *  v - q * w + g * (-sin(theta)) + X/m;

    % solve for v dot
    xdot(8) = p * w - r * u + g * cos(theta) * sin(phi) + Y/m;

    % solve for w dot
    xdot(9) = (q * u) - (p * v) + (g*cos(theta)*cos(phi)) + Z/m;
    
    %% Rotational derivatives
    gamma = Ix * Iz - Ixz^2; 
    gamma1 = (Ixz * (Ix - Iy + Iz))/gamma; 
    gamma2 = (Iz * (Iz - Iy) + Ixz^2)/gamma;
    gamma3 = Iz/gamma; 
    gamma4 = Ixz/gamma; 
    gamma5 = (Iz - Ix)/Iy;
    gamma6 = Ixz/Iy;
    gamma7 = (Ix * (Ix - Iy) + Ixz^2)/gamma; 
    gamma8 = Ix/gamma;

    % solve for p dot
    xdot(10) = gamma1 * p * q - gamma2 * q * r + gamma3 * L + gamma4 * N;

    % solve for q dot
    xdot(11) = gamma5 * p * r - gamma6 * (p^2 - r^2) + M/Iy; 

    % solve for r dot
    xdot(12) = gamma7 * p * q - gamma1 * q * r + gamma4 * L + gamma8 * N; 

end
