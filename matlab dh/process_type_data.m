clear all;
clc;
hold on;

imu = importdata('teste_medio');
%%% packet -> ACC X, ACCEL Y, ACC Z, TEMP, GYR X, GIR Y, GIR Z AND POT.

% POT 142 -> ≃ 1,6 V
x = 1:1:length(imu);
window=100;
gyro = movmean(imu(:,5),window);
plot(x,gyro); % angulo em x a partir da aceleração
plot(x,imu(:,15));

legend('Gyroscopio raw','potenciômetro');
grid on
hold off;


