Data1_Lenkradwinkel_0_mapped = Data1_Lenkradwinkel_0(8:10:end);
scatter (Data1_ay_0, Data1_Lenkradwinkel_0_mapped);
xlabel('Lateral Acceleration');
ylabel('Steering Angle');
title('Steering Angle vs Lateral Acceleration');
grid on;
