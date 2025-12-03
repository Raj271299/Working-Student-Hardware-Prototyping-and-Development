clc
figure; %Slip angle vs Lateral Acceleration
beta = atand(Data1_vy_0./Data1_vx_0);
beta_mapped = beta(2:10:end);
scatter (Data1_ay_0, beta_mapped);
xlabel('Lateral Acceleration');
ylabel('Slip Angle');
title('Slip Angle vs Lateral Acceleration');
grid on;

figure; %Slip angle vs time
plot(Data1_time_vy_0,beta);
xlabel('Time(s)');
ylabel('Slip Angle');
title('Slip Angle vs Time');
grid on;
