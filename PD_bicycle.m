
% NATNAEL SHEWANGIZAW ZEWGE
% March, 2017

% PD controller for a two wheel steering bicycle

clc ; clear all;

%parameters

V = 1;                  %constant velocity
l = 1;                  %length of bicycle( between wheel centers)
Kp = 7;                 %proportional gain
Kd = 5;                 %derivative gain
Ki = 0;              %optional integral gain

%initializations

x = 0;
y = 0;
phi = 0;
delta = 0;
dt = 0.1;               %timestep 
e_old = 0;              %initial error
e_integral=0;           %inititial accumulated error
x_accum = 0;            %for storing incoming x values
y_accum = 0;            %for storing incoming y values

% reference trajectory definition

x_t = 0:dt:100; %time scale
ref = 2*(1-exp(-0.1.*x_t)).*sin(x_t);

plot(x_t,ref,'b','DisplayName','Reference')
axis([0 50 -3 3])
xlabel('x')
ylabel('y')

%PID loop
for i = 1:dt:500
    
    y_desired = 2*(1-exp(-0.1*x))*sin(x);
    
    error = y_desired - y;
    e_dot = (error - e_old)/dt;
    e_integral = e_integral + error;
    
    % kinematic equations
    
    X_dot = V * cos(phi+delta);
    Y_dot = V * sin(phi+delta);
    phi_dot = V/l*sin(delta);
    delta_dot = Kp*error + Ki*e_integral + Kd*e_dot;
    
    e_old = error;
    
  %update position state variables
  
    x = x + X_dot*dt;
    y = y + Y_dot*dt;
    phi = phi + phi_dot * dt;
    delta = delta + delta_dot*dt;
    
  % add values to accumulator
    
    x_accum = horzcat(x_accum,x);
    y_accum = horzcat(y_accum,y);
  
     hold on
     grid on
     plot(x_accum,y_accum,'b--','DisplayName','Response')
     hold off
     title('Tracking Performance');
     legend('show')
      
     if x > 50 
            break;
     end
  
end



