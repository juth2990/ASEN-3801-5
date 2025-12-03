function PlotAircraftSim(time, aircraft_state_array, control_input,fig, col)

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
   plot(time, control_input(1,:), col)
   hold on
   ylabel('Elevator')
   title('Control Inputs')
   subplot(4,1,2)
   hold on
   plot(time, control_input(2,:), col)
   ylabel('Aileron')
   subplot(4,1,3)
   hold on
   plot(time, control_input(3,:), col)
   ylabel('Rudder')
   subplot(4,1,4)
   hold on
   plot(time, control_input(4,:), col)
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