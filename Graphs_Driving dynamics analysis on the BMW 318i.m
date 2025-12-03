%% 2024-12-11_BMW_circle.mat
clear all; close all; clc
load('2024-12-11_BMW_circle_3.mat');

%Steering angle over lateral acceleration
x = Data1_ay_0;
y = Data1_Lenkradwinkel_0(1:10:end-5);

max_ay = max(x)                             %maximum lateral acceleration [g]
steer_angle_max_ay = y(find(x==max(x)))     %steering alngle at max lateral acceleration [º]

p = polyfit(x(1:900), y(1:900), 1);   %for test #5 we use 500:1000, we use 350:1000 for the rest
px = x(1:900);
py = polyval(p, px);

self_steering_gradient = p(1)               %self steering gradient [deg/m*s^2]

figure
scatter(x, y,10,'red','filled')
hold on
plot(px,py,'b',LineWidth=2)
title('Steering wheel angle over a_y')
xlabel('a_y[g]')
ylabel('Steering wheel angle [º]')
scatter(max(x), y(find(x==max(x))),'blue','LineWidth',2)
ylim([0 max(y)+20])
legend('','EG_L',location='best')
grid on
hold off
saveas(gcf,'Steering angle over lateral acceleration.png')

%Angle of deviation over ay
beta = atan(Data1_vy_0./Data1_vx_0);
x = Data1_ay_0;
y = beta(1:10:end-5);

p = polyfit(x, y, 1);
px = x;
py = polyval(p, px);

angle_of_deviation_gradient = p(1)          %angle of deviation gradient[º/g]

figure
scatter(x, y,10,'red','filled')
hold on
plot(px,py,'b',LineWidth=2)
title('Angle of deviation over a_y')
xlabel('a_y[g]')
ylabel('Angle of deviation [º]')
grid on
hold off
saveas(gcf,'Angle of deviation over ay.png')

%Vx over ay
x = Data1_ay_0;
y = Data1_vx_0(1:10:end-5);

figure
scatter(x, y,10,'red','filled')
title('V_x over a_y')
xlabel('a_y[g]')
ylabel('V_x [km/h]')
grid on
saveas(gcf,'Vx over ay.png')


%Vy over ay
x = Data1_ay_0;
y = Data1_vy_0(1:10:end-5);

figure
scatter(x, y,10,'red','filled')
title('V_y over a_y')
xlabel('a_y[g]')
ylabel('V_y [km/h]')
grid on
saveas(gcf,'Vy over ay.png')


%Yaw rate over ay
x = Data1_ay_0;
y = Data1_Gierrate_0;

figure
scatter(x, y,10,'red','filled')
title('Yaw rate over a_y')
xlabel('a_y[g]')
ylabel('Yaw rate [º/s]')
grid on
saveas(gcf,'Yaw rate over ay.png')

%Angle of deviation vs vx
x = Data1_vx_0;
y = beta;

if isempty(x(find(y==min(abs(y)))))
    Vx_for_zero_angle_of_deviation = x(find(y==-min(abs(y))))                  %Vx for zero angle of deviation [km/h]
    min_beta = -min(abs(y));
else
    Vx_for_zero_angle_of_deviation = x(find(y==min(abs(y)))) 
    min_beta = min(abs(y));
end

figure
scatter(x, y,10,'red','filled')
hold on
scatter(Vx_for_zero_angle_of_deviation, min_beta,'blue','LineWidth',2)
title('Angle of deviation over V_x')
xlabel('V_x[km/h]')
ylabel('Angle of deviation [°]')
grid on
hold off
saveas(gcf,'Angle of deviation vs Vx.png')
