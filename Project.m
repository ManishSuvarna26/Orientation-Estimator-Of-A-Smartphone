clear all;
clc;
run('startup.m');
% run('meas_flat.m');

%Flat
%load('meas_flat (1).mat');
[xh,meas] = filterTemplate();
timef = meas.t;
accf = meas.acc;
gyrf = meas.gyr;
magf = meas.mag;
orientf = meas.orient;
%Acc Plot
figure()

subplot(2,1,1);

plot(timef,accf(1,:));
grid on;
hold on
plot(timef,accf(2,:));
legend('X','Y');
title('X  and Y axis');
subplot(2,1,2);
plot(timef,accf(3,:));
grid on;
title('Z - axis');
sgtitle('Acceleration readings')

% Angular velocity
figure();

subplot(3,1,1);
plot(timef,gyrf(1,:));
grid on;
title('X-axis')
subplot(3,1,2);
plot(timef,gyrf(2,:));
grid on;
title('Y-Axis');
subplot(3,1,3);
plot(timef,gyrf(3,:));
grid on;
title('Z-Axis');
sgtitle('Gyroscope Angular Velocity Readings');

%Magnetic Field;
figure();

subplot(3,1,1);
plot(magf(1,:));
grid on;
title('X-axis')
subplot(3,1,2);
plot(magf(2,:));
grid on;
title('Y-Axis');
subplot(3,1,3);
plot(magf(3,:));
grid on;
title('Z-Axis');
sgtitle('Magnetometer Readings');


%% Histogram for Gyroscope
clc;
check_ind = isnan(gyrf);
gyrfn = [];
for i = 1:length(gyrf(1,:))
    if check_ind(:,i) == zeros(3,1)
        gyrfn = [gyrfn gyrf(:,i)];
    end
end
 mu_gyr = mean(gyrfn,2); cov_gyr = cov(gyrfn');
 tit = {'X-axis','Y-axis','Z-axis'};
 figure()
 for i = 1:3
     %[x, y] = normpdf2(mu_gyr(i,:),cov_gyr(i,i),3,100);
    subplot(3,1,i)
    histogram(gyrfn(i,:));
    %hold on;
    %plot(x,y,'LineWidth',2);
    %xlim([min(x) max(x)]);
    %legend('Histogram','Normal Probability Curve');
    grid on;
    title(tit(i));
    sgtitle('Histogram for Gyroscope Readings');
 end

%% Histogram for Accelerometer
 
 clc;
check_ind = isnan(accf);
accn = zeros(3,1);
for i = 1:length(accf(1,:))
    if check_ind(:,i) == zeros(3,1)
        accn = [accn accf(:,i)];
    end
end
 mu_acc = mean(accn,2); cov_acc = cov(accn');
 tit = {'X-axis','Y-axis','Z-axis'};
 figure()
 for i = 1:3
    subplot(3,1,i)
    histogram(accn(i,:));
    grid on;
    title(tit(i));
    sgtitle('Histogram for Accelerometer Readings');
 end
 
 %% Histogram for Magnetometer
 
 clc;
check_ind = isnan(magf);
magn = zeros(3,1);
for i = 1:length(magf(1,:))
    if check_ind(:,i) == zeros(3,1)
        magn = [magn magf(:,i)];
    end
end
 mu_mag = mean(magn,2); cov_mag = cov(magn');
 m0 = [0, sqrt(mu_mag(1)^2 + mu_mag(2)^2), mu_mag(3)]'; 
 tit = {'X-axis','Y-axis','Z-axis'};
 figure(6)
 for i = 1:3

    subplot(3,1,i)
    histogram(magn(i,:));
    
    grid on;
    title(tit(i));
    sgtitle('Histogram for Magnetometer Readings');
 end

 %% TASK 11
clc;
clear all;


[xh,meas] = SuvarnaFilter();
xh = xh.x;
or = meas.orient;
for i = 1:length(xh(1,:))
    xh_eul(:,i) =(180/pi)*( quat2eul(xh(:,i)')');%Estimates in form euler angles
    or_eul(:,i) = (180/pi)*(quat2eul(or(:,i)')');
end
tit = {'Roll','Pitch','Yaw'};
figure()

for i = 1:3
    subplot(3,1,i)
    plot(meas.t,xh_eul(i,:),'LineWidth',2);
    hold on
    plot(meas.t,or_eul(i,:),'LineWidth',2);
    
    title(tit(i));
    legend('Own','Google');
    sgtitle('Euler angles by Own Filter vs. Google Filter');
end








