wps = [0 0 0;
      20 0 5;
      20 5 5;
      0  5 5;
      0  0 0];

vels = [2 0 0;
        2 0 0;
       -2 0 0;
       -2 0 0;
        2 0 0];

t = cumsum([0 20/2 5*pi/2/2 20/2 5*pi/2/2]');

eulerAngs = [0 0 0;
             0 0 0;
           180 0 0;
           180 0 0;
             0 0 0]; % Angles in degrees.
% Convert Euler angles to quaternions.
quats = quaternion(eulerAngs,"eulerd","ZYX","frame");

fs = 100;

traj = waypointTrajectory(wps,SampleRate=fs, ...
        Velocities=vels,...
        TimeOfArrival=t,...
        Orientation=quats);

[pos, orient, vel, acc, angvel] = traj();
i = 1;

spf = traj.SamplesPerFrame;
while ~isDone(traj)
    idx = (i+1):(i+spf);
    [pos(idx,:), orient(idx,:), ...
        vel(idx,:), acc(idx,:), angvel(idx,:)] = traj();
    i = i+spf;
end

plot(pos(:,1),pos(:,2), wps(:,1),wps(:,2), "--o")
xlabel("X (m)")
ylabel("Y (m)")
zlabel("Z (m)")
legend({"Trajectory", "Waypoints"})
axis equal
plot(pos(:,3))