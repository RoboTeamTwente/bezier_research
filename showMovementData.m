function showMovementData(movementData)
% movementData: struct with fields:
% --------------------------------
% Vel: Velocity       [dXdT; 
%                   dYdT]
% Acc: Acceleration   [d2XdT2; 
%                   d2YdT2]
% Ang: Orientation

V = movementData.Vel;
A = movementData.Acc;
ang = movementData.Ang;

% Velocity
tV = linspace(0,1,length(V(1,:)));
subplot(2,2,1)
plot(tV,V(1,:),'.',tV,V(2,:),'.',tV,sqrt(V(1,:).^2+V(2,:).^2),'.');
ylabel('Velocity [cm/a.u.]')
xlabel('Normalized time')
grid on
legend('x','y','total','location','best')

% Acceleration
tA = linspace(0,1,length(A(1,:)));
subplot(2,2,3)
plot(tA,A(1,:),'.',tA,A(2,:),'.',tA,sqrt(A(1,:).^2+A(2,:).^2),'.');
ylabel('Acceleration [cm/a.u.]')
xlabel('Normalized time')
grid on
legend('x','y','total','location','best')

% Orientation
tAng = linspace(0,1,length(V(1,:)));
subplot(2,2,2)
plot(tAng,ang,'.');
ylabel('Orientation [rad]')
xlabel('Normalized time')
grid on

% Angular velocity
angV = diff(ang);
tAngV = linspace(0,1,length(angV(1,:)));
subplot(2,2,4)
plot(tAngV,angV,'.');
ylabel('Angular velocity [rad/a.u.]')
xlabel('Normalized time')
grid on
end