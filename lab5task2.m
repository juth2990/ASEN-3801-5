% Contributors: Mansi Pahade
% Course: ASEN 3801 Lab 5
% File Name: lab5task2.m
% Date Created: 11/11/2025

clear; clc; close all;

ttwistor

%% TEST CASE 1
% initial state - all 0 except for h = 1609.34 m and u = 21 m/s
aircraft_state = [0;
      0;
      -1609.34;
      0;
      0;
      0;
      21;
      0;
      0;
      0;
      0;
      0]; 

fig = [1 2 3 4 5 6]
col = 'b'

% control surfaces all set to 0
aircraft_surfaces = [0; 0; 0; 0]; 

% wind angles all set to 0
wind_inertial = [0; 0; 0];

% time span
tspan = [0 120];

% run ode45 with EOM function
ode = @(t, x) AircraftEOM(t, x, aircraft_surfaces, wind_inertial, aircraft_parameters); 
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8); 
[t, x] = ode45(ode, tspan, aircraft_state, options); 

% % % plots
% % % % figure('Position', [50 50 1400 900], 'Name', 'Test Case 1: Near Trim Condition');
% % figure
% % % 3D trajectory
% % subplot(3,4,1); 
% % plot3(x(:,1), x(:,2), x(:,3), 'b-', 'LineWidth', 1.5); 
% % xlabel('X (m)'); 
% % ylabel('Y (m)');
% % zlabel('Altitude (m)'); 
% % title('3D Flight Path')
% % grid on; 

PlotAircraftSim(t, x', aircraft_surfaces,fig, col)

function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)

    % aircraft_state = [xi, yi, zi, roll, pitch, yaw, u, v, w, p, q, r]
    % extract states
    x = aircraft_state(1);
    y = aircraft_state(2);
    z = aircraft_state(3);
    phi = aircraft_state(4); 
    theta = aircraft_state(5);
    psi = aircraft_state(6);
    u = aircraft_state(7);
    v = aircraft_state(8);
    w = aircraft_state(9);
    p = aircraft_state(10);
    q = aircraft_state(11);
    r = aircraft_state(12); 
    
    % aircraft_surfaces = [de da dr dt];
    % extract control surfaces
    de = aircraft_surfaces(1); 
    da = aircraft_surfaces(2);
    dr = aircraft_surfaces(3);
    dt = aircraft_surfaces(4);
    

    [T,a,P,rho] = atmoscoesa(-z);
    g = aircraft_parameters.g; 
    m = aircraft_parameters.m;
    
    % calculate aero forces and moments
    [aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, rho, aircraft_parameters);
    
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

    I = [Ix 0 Ixz;0 Iy 0; Ixz 0 Iz];

    % initialize xdot vector
    xdot = zeros(12,1); 
    
    %% Kinematic Equations: 
    
    pE = TransformFromInertialToBody(I, aircraft_state); 
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

function B = TransformFromInertialToBody(I,aircraft_state)

    % aircraft_state

    phi = aircraft_state(4); 
    theta = aircraft_state(5);
    psi = aircraft_state(6); 
    
    % rotation matrix
    cphi = cos(phi);        sphi = sin(phi); 
    ctheta = cos(theta);    stheta = sin(theta); 
    cpsi = cos(psi);        spsi = sin(psi); 
    
    R_BI = [ctheta * cpsi, (sphi*stheta*cpsi) - (cphi*spsi), (cphi*stheta*cpsi) + (sphi*spsi);
            ctheta*spsi,   (sphi*stheta*spsi) + (cphi*cpsi), (cphi*stheta*spsi) - (sphi*cpsi);
            -(stheta),     stheta*ctheta,                    cphi*ctheta]; 
    
    B = R_BI * I; 

end 

function [aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters)
    
    
    % aircraft_state = [xi, yi, zi, roll, pitch, yaw, u, v, w, p, q, r]
    %   NOTE: The function assumes the veolcity is the air relative velocity
    %   vector. When used with simulink the wrapper function makes the
    %   conversion.
    %
    % aircraft_surfaces = [de da dr dt];
    %
    % Lift and Drag are calculated in Wind Frame then rotated to body frame
    % Thrust is given in Body Frame
    % Sideforce calculated in Body Frame
    
    
    %%% redefine states and inputs for ease of use
    ap = aircraft_parameters;
    
    wind_body = TransformFromInertialToBody(wind_inertial, aircraft_state);
    air_rel_vel_body = aircraft_state(7:9,1) - wind_body;
    
    [wind_angles] = WindAnglesFromVelocityBody(air_rel_vel_body);
    V = wind_angles(1,1);
    beta = wind_angles(2,1);
    alpha = wind_angles(3,1);
    
    p = aircraft_state(10,1);
    q = aircraft_state(11,1);
    r = aircraft_state(12,1);
    
    de = aircraft_surfaces(1,1);
    da = aircraft_surfaces(2,1);
    dr = aircraft_surfaces(3,1);
    dt = aircraft_surfaces(4,1);
    
    alpha_dot = 0;
    
    %Q = ap.qbar;
    Q = 0.5*density*V*V;
    
    sa = sin(alpha);
    ca = cos(alpha);
    
    %%% determine aero force coefficients
    CL = ap.CL0 + ap.CLalpha*alpha + ap.CLq*q*ap.c/(2*V) + ap.CLde*de;
    %CD = ap.CD0 + ap.CDalpha*alpha + ap.CDq*q*ap.c/(2*V) + ap.CDde*de;
    CD = ap.CDpa + ap.K*CL*CL;
    
    CX = -CD*ca + CL*sa;
    CZ = -CD*sa - CL*ca;
    
    CY = ap.CY0 + ap.CYbeta*beta + ap.CYp*p*ap.b/(2*V) + ap.CYr*r*ap.b/(2*V) + ap.CYda*da + ap.CYdr*dr;
    
    %%Thrust = .5*density*ap.Sprop*ap.Cprop*((ap.kmotor*dt)^2 - V^2);
    Thrust = density*ap.Sprop*ap.Cprop*(V + dt*(ap.kmotor - V))*dt*(ap.kmotor-V); %%Changed 5/2/15;New model as described in http://uavbook.byu.edu/lib/exe/fetch.php?media=shared:propeller_model.pdf
    
    
    %%% determine aero forces from coeffficients 
    X = Q*ap.S*CX + Thrust;
    Y = Q*ap.S*CY;
    Z = Q*ap.S*CZ;
    
    aero_forces = [X;Y;Z];
    
    %%% determine aero moment coefficients
    Cl = ap.b*[ap.Cl0 + ap.Clbeta*beta + ap.Clp*p*ap.b/(2*V) + ap.Clr*r*ap.b/(2*V) + ap.Clda*da + ap.Cldr*dr];
    Cm = ap.c*[ap.Cm0 + ap.Cmalpha*alpha + ap.Cmq*q*ap.c/(2*V) + ap.Cmde*de];
    Cn = ap.b*[ap.Cn0 + ap.Cnbeta*beta + ap.Cnp*p*ap.b/(2*V) + ap.Cnr*r*ap.b/(2*V) + ap.Cnda*da + ap.Cndr*dr];
    
    %%% determine aero moments from coeffficients
    aero_moments = Q*ap.S*[Cl; Cm; Cn];%[l;m;n];

end

function [wind_angles] = WindAnglesFromVelocityBody(air_rel_vel_body)

u = air_rel_vel_body(1); 
v = air_rel_vel_body(2); 
w = air_rel_vel_body(3); 

V = sqrt(u^2 + v^2 + w^2); 
beta = asin(v/V); 
alpha = atan2(w, u); 

wind_angles = [V; beta; alpha]; 

end

function PlotAircraftSim(time, aircraft_state_array, control_input,fig, col)
   

   control_input_array(1,:) = zeros(1,length(time))+control_input(1);
   control_input_array(2,:) = zeros(1,length(time))+control_input(2);
   control_input_array(3,:) = zeros(1,length(time))+control_input(3);
   control_input_array(4,:) = zeros(1,length(time))+control_input(4);

   % position subplot
   figure(fig(1))
   hold on
   subplot(3,1,1)
   hold on
   plot(time, aircraft_state_array(1,:), col)
   ylabel('x_E [m]')
   title('Inertial Position')
   subplot(3,1,2)
   hold on
   plot(time, aircraft_state_array(2,:), col)
   ylabel('y_E [m]')
   subplot(3,1,3)
   hold on
   plot(time, aircraft_state_array(3,:),col)
   ylabel('z_E [m]')
   xlabel('Time [s]')
   saveas(gcf,'position.jpg');

   % Euler Angles subplot
   figure(fig(2))
   hold on
   subplot(3,1,1)
   hold on
   plot(time, aircraft_state_array(4,:), col)
   ylabel('\phi [rad]')
   title('Euler Angles')
   subplot(3,1,2)
   hold on
   plot(time, aircraft_state_array(5,:), col)
   ylabel('\theta [rad]')
   subplot(3,1,3)
   hold on
   plot(time, aircraft_state_array(6,:), col)
   ylabel('\psi [rad]')
   xlabel('Time [s]')
   saveas(gcf,'Euler.jpg');
 
   % Body-Frame Velocity subplot
   figure(fig(3))
   hold on
   subplot(3,1,1)
   hold on
   plot(time, aircraft_state_array(7,:), col)
   ylabel('u_E [m/s]')
   title('Body-Frame Velocity')
   subplot(3,1,2)
   hold on
   plot(time, aircraft_state_array(8,:), col)
   ylabel('v_E [m/s]')
   subplot(3,1,3)
   hold on
   plot(time, aircraft_state_array(9,:), col)
   ylabel('w_E [m/s]')
   xlabel('Time [s]')
   saveas(gcf,'Velocity.jpg');

   % Angular Velocity subplot
   figure(fig(4))
   hold on
   subplot(3,1,1)
   hold on
   plot(time, aircraft_state_array(10,:), col)
   ylabel('p [rad/s]')
   title('Angular Velocity')
   subplot(3,1,2)
   hold on
   plot(time, aircraft_state_array(11,:), col)
   ylabel('q [rad/s]')
   subplot(3,1,3)
   hold on
   plot(time, aircraft_state_array(12,:), col)
   ylabel('r [rad/s]')
   xlabel('Time [s]')
   saveas(gcf,'Angular Velocity.jpg');

   % control subplot
   figure(fig(5))
   hold on
   subplot(4,1,1)
   hold on
   plot(time, control_input_array(1,:), col)
   hold on
   ylabel('Elevator')
   title('Control Inputs')
   subplot(4,1,2)
   hold on
   plot(time, control_input_array(2,:), col)
   ylabel('Aileron')
   subplot(4,1,3)
   hold on
   plot(time, control_input_array(3,:), col)
   ylabel('Rudder')
   subplot(4,1,4)
   hold on
   plot(time, control_input_array(4,:), col)
   ylabel('Thrust')
   
   saveas(gcf,'Control.jpg');

   % 3D Flight Path
   figure(fig(6));
   plot3(aircraft_state_array(1,:), aircraft_state_array(2,:), -aircraft_state_array(3,:), col, 'LineWidth', 1.5)
   hold on
   plot3(aircraft_state_array(1,1), aircraft_state_array(2,1), -aircraft_state_array(3,1), 'go', 'MarkerFaceColor', 'g')
   hold on
   plot3(aircraft_state_array(1,end), aircraft_state_array(2,end), -aircraft_state_array(3,end), 'ro', 'MarkerFaceColor', 'r')
   xlabel('x_E [m]')
   ylabel('y_E [m]')
   zlabel('z_E [m]')
   grid on
   axis equal
   title('3D Flight Path')
   saveas(gcf,'3D.jpg');

end