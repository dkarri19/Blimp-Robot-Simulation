clc
clear variables
close all

% Note that in this simulation the agents need to know the position of the
% target. Press ctrl C to terminate the simulation while running.

% Declare The Target and Robot Initial Position Here
xtarget = -1;
ytarget = 1;

globalx(1:2)  = 10;
globaly(1:2)  = 10;

% Declare the PID Gains Here
kp_a  = 7;
kd_a  = 7;

velocitybodyx(1:2) = 0;
velocitybodyy(1:2) = 0;
sideslipang(1:2)  = 0;
angveldot(1:2) = 0;
angvel(1:2)  = 0;
heading(1:2)  = 0;  
Dragx(1:2)  = 0;
Sideslip(1:2)  = 0;
Moment(1:2)  = 0;

resultantvel(1:2)  = 0;

force     = 0.2;
densityair     = 1.255;
crossarea     = 0.19;
length     = 0.6223;
breadth   = 0.2032;
height    = 0.2921;
mass     = 0.20;
inertia     = (length^2+breadth^2)*mass/2;

com   = 0;
co    = 0;
cod   = 3.1092;
cd    = -0.4828;
cs    = 0.0946;
cm    = 0.0658;
K     = 0.0126;

T(1:2)  = 1;
pid_a(1:2) = 0;
t = 0.001;

xgrabber(1:2) = globalx(2)+length*cos(heading(2));
ygrabber(1:2) = globaly(2)+length*sin(heading(2));
xtail(1:2) = globalx(2)-length*cos(heading(2));
ytail(1:2) = globaly(2)-length*sin(heading(2));
phi(1:2) = 0;
errorx(1:2) = xtarget-xgrabber(2);
errory(1:2) = ytarget-ygrabber(2);
targetdistance(1:2)  = sqrt(errorx(2)^2+errory(2)^2);
errorang(1:2) = 0;
mode(1:2) = 1;
n = 2;
radius_ball = 0.2;
graber_width = 0.3;
radius = 2;
grabber_depth = 0.2;
tail_length = 0.2;
tail_actuator_limit = 45;
pid_operating_range = 60;
distance_limit = 6;
Capture_distance = 0.2;

% d1


if targetdistance(n-1)<distance_limit
    while targetdistance(n-1)<=distance_limit
        
        %% Graphis for the grabber and the robot
        xgrabber(n) = globalx(n)+(length+grabber_depth+0.3)*cos(heading(n));
        ygrabber(n) = globaly(n)+(length+grabber_depth+0.3)*sin(heading(n));
        grabberx1(n) = xgrabber(n)+graber_width*cos(heading(n)+(pi/2));
        grabbery1(n) = ygrabber(n)+graber_width*sin(heading(n)+(pi/2));
        grabberx2(n) = xgrabber(n)-graber_width*cos(heading(n)+(pi/2));
        grabbery2(n) = ygrabber(n)-graber_width*sin(heading(n)+(pi/2));
        xtail(n) = globalx(n)-length*cos(heading(n));
        ytail(n) = globaly(n)-length*sin(heading(n));
        errorx(n) = xtarget-xgrabber(n);
        errory(n) = ytarget-ygrabber(n);
        
        %% Getting Distance and heading values for the target
        targetdistance(n)  = sqrt(errorx(n)^2+errory(n)^2);
        phi(n) = atan2(errory(n),errorx(n));

        if phi(n) < 0
            phi(n) = phi(n) + 2*pi;
        elseif phi(n) >= 2*pi
            phi(n) = phi(n) - 2*pi;
        end

        %% Pid for heading correction
        errorang(n) = phi(n) - heading(n);
        e1diff(n) = (errorang(n) - errorang(n-1))/t;
        pid_a(n) = 0;

        if errorang(n) < 0
            errorang(n) = errorang(n) + 2*pi;
        elseif errorang(n) >= 2*pi
            errorang(n) = errorang(n) - 2*pi;
        end

        mode(n) = 1;

        %% Tail Propeller Graphics
        propellerx(n) = xtail(n)+tail_length*cos(pi-pid_a(n)+heading(n));
        propellery(n) = ytail(n)+tail_length*sin(pi-pid_a(n)+heading(n));

        %% Dynamics
        vdx1(n)  = (force*cos(pid_a(n))-Dragx(n)*cos(sideslipang(n))-Sideslip(n)*sin(sideslipang(n)))/mass;
        velocitybodyx(n+1) = vdx1(n)*t+velocitybodyx(n);
        vdy1(n)  = (force*sin(pid_a(n))-Dragx(n)*sin(sideslipang(n))+Sideslip(n)*cos(sideslipang(n)))/mass;
        velocitybodyy(n+1) = vdy1(n)*t+velocitybodyy(n);
        sideslipang(n+1)  = atan(velocitybodyy(n+1)/velocitybodyx(n+1));
        resultantvel(n+1)  = sqrt((velocitybodyx(n+1)^2)+(velocitybodyy(n+1)^2));

        angveldot(n)   = (1/inertia)*(length*force*sin(pid_a(n))+Moment(n)-K*angvel(n));
        angvel(n+1)  = angveldot(n)*t+angvel(n);
        pd1(n)   = angvel(n);
        heading(n+1)  = pd1(n)*t+heading(n);
        if heading(n+1) == pi
            heading(n+1) = -pi;
        end

        Dragx(n+1)  = densityair*(resultantvel(n+1)^2)*(cod+cd*(sideslipang(n+1)^2))*crossarea/2;
        Moment(n+1)  = densityair*(resultantvel(n+1)^2)*(com+cm*sideslipang(n+1))*crossarea/2;
        Sideslip(n+1)  = densityair*(resultantvel(n+1)^2)*(co+cs*sideslipang(n+1))*crossarea/2;

        xd1(n)   = velocitybodyx(n)*cos(heading(n))-velocitybodyy(n)*sin(heading(n));
        globalx(n+1)  = xd1(n)*t+globalx(n);
        yd1(n)   = velocitybodyy(n)*cos(heading(n))+velocitybodyx(n)*sin(heading(n));
        globaly(n+1)  = yd1(n)*t+globaly(n);

        T(n+1)  = n+1;

        n = n+1;
        T(n) = n;
    end
end
    %elseif targetdistance(n-1)>=distance_limit
    while targetdistance(n-1)>Capture_distance
        %% Update for the Graphics
        xgrabber(n) = globalx(n)+(length+grabber_depth+0.3)*cos(heading(n));
        ygrabber(n) = globaly(n)+(length+grabber_depth+0.3)*sin(heading(n));
        grabberx1(n) = xgrabber(n)+graber_width*cos(heading(n)+(pi/2));
        grabbery1(n) = ygrabber(n)+graber_width*sin(heading(n)+(pi/2));
        grabberx2(n) = xgrabber(n)-graber_width*cos(heading(n)+(pi/2));
        grabbery2(n) = ygrabber(n)-graber_width*sin(heading(n)+(pi/2));
        xtail(n) = globalx(n)-length*cos(heading(n));
        ytail(n) = globaly(n)-length*sin(heading(n));
        errorx(n) = xtarget-xgrabber(n);
        errory(n) = ytarget-ygrabber(n);
        
        %% distance and heading values to the target
        targetdistance(n)  = sqrt(errorx(n)^2+errory(n)^2);
        phi(n) = atan2(errory(n),errorx(n));

        if phi(n) < 0
            phi(n) = phi(n) + 2*pi;
        elseif phi(n) >= 2*pi
            phi(n) = phi(n) - 2*pi;
        end

        %% Error for the heading and PID
        errorang(n) = phi(n) - heading(n);
        e1diff(n) = (errorang(n) - errorang(n-1))/t;
        %pid_a(n) = 0.4*tanh(4*errorang(n));
        pid_a(n) = kp_a*errorang(n)+kd_a*e1diff(n);
        if pid_a(n) > tail_actuator_limit*pi/180
            pid_a(n) = tail_actuator_limit*pi/180;
        end
        if pid_a(n) < -tail_actuator_limit*pi/180
            pid_a(n) = -tail_actuator_limit*pi/180;
        end

        if errorang(n) < 0
            errorang(n) = errorang(n) + 2*pi;
        elseif errorang(n) >= 2*pi
            errorang(n) = errorang(n) - 2*pi;
        end

        if targetdistance(n) < distance_limit
            if errorang(n) < pid_operating_range*pi/180 && errorang(n) > -pid_operating_range*pi/180
                mode(n) = 2;
                pid_a(n) = pid_a(n);
            elseif errorang(n) > pid_operating_range*pi/180 || errorang(n) < -pid_operating_range*pi/180
                mode(n) = 3;
                pid_a(n) = pid_a(n);
            end
        end

        %% Graphics for Tail propeller
        propellerx(n) = xtail(n)+tail_length*cos(pi-pid_a(n)+heading(n));
        propellery(n) = ytail(n)+tail_length*sin(pi-pid_a(n)+heading(n));

        %% Robot Dynamics
        vdx1(n)  = (force*cos(pid_a(n))-Dragx(n)*cos(sideslipang(n))-Sideslip(n)*sin(sideslipang(n)))/mass;
        velocitybodyx(n+1) = vdx1(n)*t+velocitybodyx(n);
        vdy1(n)  = (force*sin(pid_a(n))-Dragx(n)*sin(sideslipang(n))+Sideslip(n)*cos(sideslipang(n)))/mass;
        velocitybodyy(n+1) = vdy1(n)*t+velocitybodyy(n);
        sideslipang(n+1)  = atan(velocitybodyy(n+1)/velocitybodyx(n+1));
        resultantvel(n+1)  = sqrt((velocitybodyx(n+1)^2)+(velocitybodyy(n+1)^2));

        angveldot(n)   = (1/inertia)*(length*force*sin(pid_a(n))+Moment(n)-K*angvel(n));
        angvel(n+1)  = angveldot(n)*t+angvel(n);
        pd1(n)   = angvel(n);
        heading(n+1)  = pd1(n)*t+heading(n);
        if heading(n+1) == pi
            heading(n+1) = -pi;
        end

        Dragx(n+1)  = densityair*(resultantvel(n+1)^2)*(cod+cd*(sideslipang(n+1)^2))*crossarea/2;
        Moment(n+1)  = densityair*(resultantvel(n+1)^2)*(com+cm*sideslipang(n+1))*crossarea/2;
        Sideslip(n+1)  = densityair*(resultantvel(n+1)^2)*(co+cs*sideslipang(n+1))*crossarea/2;

        xd1(n)   = velocitybodyx(n)*cos(heading(n))-velocitybodyy(n)*sin(heading(n));
        globalx(n+1)  = xd1(n)*t+globalx(n);
        yd1(n)   = velocitybodyy(n)*cos(heading(n))+velocitybodyx(n)*sin(heading(n));
        globaly(n+1)  = yd1(n)*t+globaly(n);

        T(n+1)  = n+1;

        n = n+1;
        T(n) = n;
        
        %% Timer to end the simulation
        if targetdistance(n-1)<=Capture_distance
            fprintf('capture success');
        end
        if n > 10^6
            break
        end
    end
%end


for n = 2:100:n-1
clf
ang = 0:0.01:2*pi;
xc = xtarget+distance_limit*cos(ang);
yc = ytarget+distance_limit*sin(ang);
xtc = xtarget+radius_ball*cos(ang);
ytc = ytarget+radius_ball*sin(ang);
figure(1)
plot(xc,yc);
hold on
plot([xtail(n-1) xgrabber(n-1)],[ytail(n-1) ygrabber(n-1)])
hold on
plot([grabberx1(n-1) grabberx2(n-1)],[grabbery1(n-1) grabbery2(n-1)])
hold on
plot([xtail(n-1) propellerx(n-1)],[ytail(n-1) propellery(n-1)])
hold on
plot(xtc,ytc); xlabel('x distance'); ylabel('y distance');% xlim([-distance_limit+1 distance_limit+1]); ylim([-distance_limit+1 distance_limit+1]);
axis equal
pause(0.01)
end

figure(2)
subplot(311)
plot(T,velocitybodyx); title('Velocity Body_x'); xlabel('time (ms)'); ylabel('vel (m/s)');
subplot(312)
plot(T,velocitybodyy); title('Velocity Body_y'); xlabel('time (ms)'); ylabel('vel (m/s)');
subplot(313)
plot(T,angvel); title('Ang Velocity Body'); xlabel('time (ms)'); ylabel('ang vel (m/s)');

figure(3)
subplot(311);
plot(T,Dragx); title('drag'); xlabel('time (ms)'); ylabel('drag');
subplot(312); 
plot(T,Moment); title('moment'); xlabel('time (ms)'); ylabel('moment');
subplot(313);
plot(T,Sideslip); title('side slip'); xlabel('time (ms)'); ylabel('side slip');

figure(4)
subplot(511);
plot(T,heading*180/pi); title('heading global');  xlabel('time (ms)'); ylabel('Heading Global');
subplot(512);
plot(phi*180/pi); title('Required Heading');  xlabel('time (ms)'); ylabel('Heading Error');
subplot(513);
plot(pid_a*180/pi); title('tail angle');  xlabel('time (ms)'); ylabel('Tail Angle');
subplot(514);
plot(targetdistance); title('Target Distance');  xlabel('time (ms)'); ylabel('distance (m)');
subplot(515);
plot(mode); title('mode');  xlabel('time (ms)'); ylabel('mode');
clc
clear variables
close all

% Note that in this simulation the agents need to know the position of the
% target. Press ctrl C to terminate the simulation while running.

% Declare The Target and Robot Initial Position Here
xtarget = -1;
ytarget = 1;

globalx(1:2)  = 10;
globaly(1:2)  = 10;

% Declare the PID Gains Here
kp_a  = 7;
kd_a  = 7;

velocitybodyx(1:2) = 0;
velocitybodyy(1:2) = 0;
sideslipang(1:2)  = 0;
angveldot(1:2) = 0;
angvel(1:2)  = 0;
heading(1:2)  = 0;  
Dragx(1:2)  = 0;
Sideslip(1:2)  = 0;
Moment(1:2)  = 0;

resultantvel(1:2)  = 0;

force     = 0.2;
densityair     = 1.255;
crossarea     = 0.19;
length     = 0.6223;
breadth   = 0.2032;
height    = 0.2921;
mass     = 0.20;
inertia     = (length^2+breadth^2)*mass/2;

com   = 0;
co    = 0;
cod   = 3.1092;
cd    = -0.4828;
cs    = 0.0946;
cm    = 0.0658;
K     = 0.0126;

T(1:2)  = 1;
pid_a(1:2) = 0;
t = 0.001;

xgrabber(1:2) = globalx(2)+length*cos(heading(2));
ygrabber(1:2) = globaly(2)+length*sin(heading(2));
xtail(1:2) = globalx(2)-length*cos(heading(2));
ytail(1:2) = globaly(2)-length*sin(heading(2));
phi(1:2) = 0;
errorx(1:2) = xtarget-xgrabber(2);
errory(1:2) = ytarget-ygrabber(2);
targetdistance(1:2)  = sqrt(errorx(2)^2+errory(2)^2);
errorang(1:2) = 0;
mode(1:2) = 1;
n = 2;
radius_ball = 0.2;
graber_width = 0.3;
radius = 2;
grabber_depth = 0.2;
tail_length = 0.2;
tail_actuator_limit = 45;
pid_operating_range = 60;
distance_limit = 6;
Capture_distance = 0.2;

% d1


if targetdistance(n-1)<distance_limit
    while targetdistance(n-1)<=distance_limit
        
        %% Graphis for the grabber and the robot
        xgrabber(n) = globalx(n)+(length+grabber_depth+0.3)*cos(heading(n));
        ygrabber(n) = globaly(n)+(length+grabber_depth+0.3)*sin(heading(n));
        grabberx1(n) = xgrabber(n)+graber_width*cos(heading(n)+(pi/2));
        grabbery1(n) = ygrabber(n)+graber_width*sin(heading(n)+(pi/2));
        grabberx2(n) = xgrabber(n)-graber_width*cos(heading(n)+(pi/2));
        grabbery2(n) = ygrabber(n)-graber_width*sin(heading(n)+(pi/2));
        xtail(n) = globalx(n)-length*cos(heading(n));
        ytail(n) = globaly(n)-length*sin(heading(n));
        errorx(n) = xtarget-xgrabber(n);
        errory(n) = ytarget-ygrabber(n);
        
        %% Getting Distance and heading values for the target
        targetdistance(n)  = sqrt(errorx(n)^2+errory(n)^2);
        phi(n) = atan2(errory(n),errorx(n));

        if phi(n) < 0
            phi(n) = phi(n) + 2*pi;
        elseif phi(n) >= 2*pi
            phi(n) = phi(n) - 2*pi;
        end

        %% Pid for heading correction
        errorang(n) = phi(n) - heading(n);
        e1diff(n) = (errorang(n) - errorang(n-1))/t;
        pid_a(n) = 0;

        if errorang(n) < 0
            errorang(n) = errorang(n) + 2*pi;
        elseif errorang(n) >= 2*pi
            errorang(n) = errorang(n) - 2*pi;
        end

        mode(n) = 1;

        %% Tail Propeller Graphics
        propellerx(n) = xtail(n)+tail_length*cos(pi-pid_a(n)+heading(n));
        propellery(n) = ytail(n)+tail_length*sin(pi-pid_a(n)+heading(n));

        %% Dynamics
        vdx1(n)  = (force*cos(pid_a(n))-Dragx(n)*cos(sideslipang(n))-Sideslip(n)*sin(sideslipang(n)))/mass;
        velocitybodyx(n+1) = vdx1(n)*t+velocitybodyx(n);
        vdy1(n)  = (force*sin(pid_a(n))-Dragx(n)*sin(sideslipang(n))+Sideslip(n)*cos(sideslipang(n)))/mass;
        velocitybodyy(n+1) = vdy1(n)*t+velocitybodyy(n);
        sideslipang(n+1)  = atan(velocitybodyy(n+1)/velocitybodyx(n+1));
        resultantvel(n+1)  = sqrt((velocitybodyx(n+1)^2)+(velocitybodyy(n+1)^2));

        angveldot(n)   = (1/inertia)*(length*force*sin(pid_a(n))+Moment(n)-K*angvel(n));
        angvel(n+1)  = angveldot(n)*t+angvel(n);
        pd1(n)   = angvel(n);
        heading(n+1)  = pd1(n)*t+heading(n);
        if heading(n+1) == pi
            heading(n+1) = -pi;
        end

        Dragx(n+1)  = densityair*(resultantvel(n+1)^2)*(cod+cd*(sideslipang(n+1)^2))*crossarea/2;
        Moment(n+1)  = densityair*(resultantvel(n+1)^2)*(com+cm*sideslipang(n+1))*crossarea/2;
        Sideslip(n+1)  = densityair*(resultantvel(n+1)^2)*(co+cs*sideslipang(n+1))*crossarea/2;

        xd1(n)   = velocitybodyx(n)*cos(heading(n))-velocitybodyy(n)*sin(heading(n));
        globalx(n+1)  = xd1(n)*t+globalx(n);
        yd1(n)   = velocitybodyy(n)*cos(heading(n))+velocitybodyx(n)*sin(heading(n));
        globaly(n+1)  = yd1(n)*t+globaly(n);

        T(n+1)  = n+1;

        n = n+1;
        T(n) = n;
    end
end
    %elseif targetdistance(n-1)>=distance_limit
    while targetdistance(n-1)>Capture_distance
        %% Update for the Graphics
        xgrabber(n) = globalx(n)+(length+grabber_depth+0.3)*cos(heading(n));
        ygrabber(n) = globaly(n)+(length+grabber_depth+0.3)*sin(heading(n));
        grabberx1(n) = xgrabber(n)+graber_width*cos(heading(n)+(pi/2));
        grabbery1(n) = ygrabber(n)+graber_width*sin(heading(n)+(pi/2));
        grabberx2(n) = xgrabber(n)-graber_width*cos(heading(n)+(pi/2));
        grabbery2(n) = ygrabber(n)-graber_width*sin(heading(n)+(pi/2));
        xtail(n) = globalx(n)-length*cos(heading(n));
        ytail(n) = globaly(n)-length*sin(heading(n));
        errorx(n) = xtarget-xgrabber(n);
        errory(n) = ytarget-ygrabber(n);
        
        %% distance and heading values to the target
        targetdistance(n)  = sqrt(errorx(n)^2+errory(n)^2);
        phi(n) = atan2(errory(n),errorx(n));

        if phi(n) < 0
            phi(n) = phi(n) + 2*pi;
        elseif phi(n) >= 2*pi
            phi(n) = phi(n) - 2*pi;
        end

        %% Error for the heading and PID
        errorang(n) = phi(n) - heading(n);
        e1diff(n) = (errorang(n) - errorang(n-1))/t;
        %pid_a(n) = 0.4*tanh(4*errorang(n));
        pid_a(n) = kp_a*errorang(n)+kd_a*e1diff(n);
        if pid_a(n) > tail_actuator_limit*pi/180
            pid_a(n) = tail_actuator_limit*pi/180;
        end
        if pid_a(n) < -tail_actuator_limit*pi/180
            pid_a(n) = -tail_actuator_limit*pi/180;
        end

        if errorang(n) < 0
            errorang(n) = errorang(n) + 2*pi;
        elseif errorang(n) >= 2*pi
            errorang(n) = errorang(n) - 2*pi;
        end

        if targetdistance(n) < distance_limit
            if errorang(n) < pid_operating_range*pi/180 && errorang(n) > -pid_operating_range*pi/180
                mode(n) = 2;
                pid_a(n) = pid_a(n);
            elseif errorang(n) > pid_operating_range*pi/180 || errorang(n) < -pid_operating_range*pi/180
                mode(n) = 3;
                pid_a(n) = pid_a(n);
            end
        end

        %% Graphics for Tail propeller
        propellerx(n) = xtail(n)+tail_length*cos(pi-pid_a(n)+heading(n));
        propellery(n) = ytail(n)+tail_length*sin(pi-pid_a(n)+heading(n));

        %% Robot Dynamics
        vdx1(n)  = (force*cos(pid_a(n))-Dragx(n)*cos(sideslipang(n))-Sideslip(n)*sin(sideslipang(n)))/mass;
        velocitybodyx(n+1) = vdx1(n)*t+velocitybodyx(n);
        vdy1(n)  = (force*sin(pid_a(n))-Dragx(n)*sin(sideslipang(n))+Sideslip(n)*cos(sideslipang(n)))/mass;
        velocitybodyy(n+1) = vdy1(n)*t+velocitybodyy(n);
        sideslipang(n+1)  = atan(velocitybodyy(n+1)/velocitybodyx(n+1));
        resultantvel(n+1)  = sqrt((velocitybodyx(n+1)^2)+(velocitybodyy(n+1)^2));

        angveldot(n)   = (1/inertia)*(length*force*sin(pid_a(n))+Moment(n)-K*angvel(n));
        angvel(n+1)  = angveldot(n)*t+angvel(n);
        pd1(n)   = angvel(n);
        heading(n+1)  = pd1(n)*t+heading(n);
        if heading(n+1) == pi
            heading(n+1) = -pi;
        end

        Dragx(n+1)  = densityair*(resultantvel(n+1)^2)*(cod+cd*(sideslipang(n+1)^2))*crossarea/2;
        Moment(n+1)  = densityair*(resultantvel(n+1)^2)*(com+cm*sideslipang(n+1))*crossarea/2;
        Sideslip(n+1)  = densityair*(resultantvel(n+1)^2)*(co+cs*sideslipang(n+1))*crossarea/2;

        xd1(n)   = velocitybodyx(n)*cos(heading(n))-velocitybodyy(n)*sin(heading(n));
        globalx(n+1)  = xd1(n)*t+globalx(n);
        yd1(n)   = velocitybodyy(n)*cos(heading(n))+velocitybodyx(n)*sin(heading(n));
        globaly(n+1)  = yd1(n)*t+globaly(n);

        T(n+1)  = n+1;

        n = n+1;
        T(n) = n;
        
        %% Timer to end the simulation
        if targetdistance(n-1)<=Capture_distance
            fprintf('capture success');
        end
        if n > 10^6
            break
        end
    end
%end


for n = 2:100:n-1
clf
ang = 0:0.01:2*pi;
xc = xtarget+distance_limit*cos(ang);
yc = ytarget+distance_limit*sin(ang);
xtc = xtarget+radius_ball*cos(ang);
ytc = ytarget+radius_ball*sin(ang);
figure(1)
plot(xc,yc);
hold on
plot([xtail(n-1) xgrabber(n-1)],[ytail(n-1) ygrabber(n-1)])
hold on
plot([grabberx1(n-1) grabberx2(n-1)],[grabbery1(n-1) grabbery2(n-1)])
hold on
plot([xtail(n-1) propellerx(n-1)],[ytail(n-1) propellery(n-1)])
hold on
plot(xtc,ytc); xlabel('x distance'); ylabel('y distance');% xlim([-distance_limit+1 distance_limit+1]); ylim([-distance_limit+1 distance_limit+1]);
axis equal
pause(0.01)
end

figure(2)
subplot(311)
plot(T,velocitybodyx); title('Velocity Body_x'); xlabel('time (ms)'); ylabel('vel (m/s)');
subplot(312)
plot(T,velocitybodyy); title('Velocity Body_y'); xlabel('time (ms)'); ylabel('vel (m/s)');
subplot(313)
plot(T,angvel); title('Ang Velocity Body'); xlabel('time (ms)'); ylabel('ang vel (m/s)');

figure(3)
subplot(311);
plot(T,Dragx); title('drag'); xlabel('time (ms)'); ylabel('drag');
subplot(312); 
plot(T,Moment); title('moment'); xlabel('time (ms)'); ylabel('moment');
subplot(313);
plot(T,Sideslip); title('side slip'); xlabel('time (ms)'); ylabel('side slip');

figure(4)
subplot(511);
plot(T,heading*180/pi); title('heading global');  xlabel('time (ms)'); ylabel('Heading Global');
subplot(512);
plot(phi*180/pi); title('Required Heading');  xlabel('time (ms)'); ylabel('Heading Error');
subplot(513);
plot(pid_a*180/pi); title('tail angle');  xlabel('time (ms)'); ylabel('Tail Angle');
subplot(514);
plot(targetdistance); title('Target Distance');  xlabel('time (ms)'); ylabel('distance (m)');
subplot(515);
plot(mode); title('mode');  xlabel('time (ms)'); ylabel('mode');
