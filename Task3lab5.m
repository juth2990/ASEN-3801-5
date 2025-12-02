%% Task 3
ttwistor 
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

fig = [1 2 3 4 5 6];
col = 'b';

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
