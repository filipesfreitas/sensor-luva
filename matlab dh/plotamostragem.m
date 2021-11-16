clear all;
clc;
importamostragem;

window_mean = 100;
%falange proximal
theta_01_filt = movmean(phi_metacarpa,100);
% falange medial
theta_02_filt = movmean(proximalphi,100);


%% Parâmetros fixos
L0 = 30;
a0 = 15;
L = [50, 31, 25]; % Falanges em mm
a =     [0,     L(1),   L(2),   L(3)];
d =     [0,     0,      0,      0];
alpha = [pi/2,   0,      0,      0];

mth_base = [rot_axis(-pi/2,2), [0;a0;L0];0,0,0,1];

start=1;
figure('units','normalized','outerposition',[0 0 1 1])
for i=start:10:length(theta_01_filt)
%% Matrizes de transformação homogênea

A0 = mth_base;
A01 = A0  * D_H_par( 0,alpha(1),d(1),a(1)); % matriz de transformação homogênea do frame A1 ao frame A2; Matriz A12
A12 = A01 * D_H_par( theta_01_filt(i),alpha(2),d(2),a(2)); % matriz de transformação homogênea do frame A2 ao frame A3; Matriz A23
A23 = A12 * D_H_par( theta_02_filt(i),alpha(3),d(3),a(3)); % matriz de transformação homogênea do frame A3 ao frame A4; Matriz A34
A34 = A23 * D_H_par( theta_02_filt(i)/2,alpha(4),d(4),a(4)); % matriz de transformação homogênea do frame A4 ao frame A5; Matriz A45

%% Sinal dos ângulos medidos
 clf
 fig.OuterPosition=[0 0 1 1];
 subplot(121),hold on
 %plot(phi_acc1(start:i)*180/pi);
 %plot(phi_acc2(start:i)*180/pi);
 %plot(phi_acc3(start:i)*180/pi);
 plot(theta_01_filt(start:i));
 plot(theta_02_filt(start:i));
  legend('Ângulo phi referência','Ângulo phi articulação MCF','Ângulo phi articulação IFP','Ângulo theta articulação IFP', 'Ângulo theta articulação IFD','Orientation','vertical')
  xlabel('Amostra '),ylabel('Ângulo (°)')
axis([0 inf,-270 500])
grid on
%% Desenhar as linhas que representam os links  entre juntas
subplot(122),hold on, grid on
view(0,0)
pbase = [0,0,0];
p0=A0(1:3,4);
p1=A01(1:3,4);
p2=A12(1:3,4);
p3=A23(1:3,4);
p4=A34(1:3,4);
cord_system(A0 ,p0,'A0');
cord_system(A01,p1,'A1');
cord_system(A12,p2,'A2');
cord_system(A23,p3,'A3');
cord_system(A34,p4,'A4');

 x = [p0(1);p1(1)];
 y = [p0(2);p1(2)];
 z = [p0(3);p1(3)];
 plot3(x,y,z ,'--','Color', [0 0 0]);
 x = [p1(1);p2(1)];
 y = [p1(2);p2(2)];
 z = [p1(3);p2(3)];
 plot3(x,y,z ,'Color', [0 0 0],'LineWidth',3);
 x = [p2(1);p3(1)];
 y = [p2(2);p3(2)];
 z = [p2(3);p3(3)];
 plot3(x,y,z ,'Color', [0 0 0],'LineWidth',3);
 x = [p3(1);p4(1)];
 y = [p3(2);p4(2)];
 z = [p3(3);p4(3)];
 plot3(x,y,z ,'Color', [0 0 0],'LineWidth',3);

xlabel('x (mm)'),ylabel('y'),zlabel('z (mm)')
axis([-40 80, -25 25 0 140])
pause(.001);
end


%% Fim 
