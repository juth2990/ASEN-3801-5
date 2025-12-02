%% Task 1
%% Task 2
clear; clc; close all;

% TEST CASE 1
% initial state - all 0 except for h = 1609.34 m and u = 21 m/s
ttwistor

x0 = [0;
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

% control surfaces all set to 0
aircraft_surfaces = [0; 0; 0; 0]; 

% wind angles all set to 0
wind_inertial = [0; 0; 0];

% time span
tspan = [0 120];

% run ode45 with EOM function
ode = @(t, x) AircraftEOM(t, x0, aircraft_surfaces, wind_inertial, aircraft_parameters); 
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8); 
[t, x] = ode45(ode, tspan, x0, options); 

% plots
figure('Position', [50 50 1400 900], 'Name', 'Test Case 1: Near Trim Condition');

% 3D trajectory
subplot(3,4,1); 
plot3(x(:,1), x(:,2), x(:,3), 'b-', 'LineWidth', 1.5); 
xlabel('X (m)'); 
ylabel('Y (m)');
zlabel('Altitude (m)'); 
title('3D Flight Path')
grid on; axis eq
%% Task 3
aircraft_state = [0;
      0;
      -1800;
      0;
      0.0278;
      0;
      20.99;
      0;
      0.5837;
      0;
      0;
      0]; 

fig = [1 2 3 4 5 6]
col = 'b'

% control surfaces all set to 0
aircraft_surfaces = [0.1079; 0; 0; 0.3182]; 

% wind angles all set to 0
wind_inertial = [0; 0; 0];

% time span
tspan = [0 3];

doublet_time = 0.25;
doublet_size = 15*(2*pi)/180; % degrees

% run ode45 with EOM function
ode = @(t, x) AircraftEOMDoublet(t, aircraft_state, aircraft_surfaces, doublet_size,doublet_time, wind_inertial, aircraft_parameters); 
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8); 
[t, x] = ode45(ode, tspan, aircraft_state, options); 