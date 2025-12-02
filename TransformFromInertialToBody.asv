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