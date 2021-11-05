test = csvread('fixa.txt');
sensor1 = test(1:end,3:8);
sensor2 = test(1:end,11:end);
theta1 = acos(sensor1(:,3)/sqrt(sensor1(:,1).^2+sensor1(:,2).^2+sensor1(:,3).^2));
hold on;
plot(theta1);
plot(sensor2(:,1));