function xdot = AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces, doublet_size,doublet_time, wind_inertial, aircraft_parameters)
deltaE = aircraft_surfaces(1);

if 0 < time && time <= doublet_time
    deltaE = deltaE + doublet_size;
elseif doublet_time < time && time <= 2*doublet_time
        deltaE = deltaE - doublet_size;
else
    deltaE = deltaE;

end