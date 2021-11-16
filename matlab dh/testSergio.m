clear all;
clc;
GET_SENSOR_DATA_FROM_FILE;

%% Adquire dados individuais
ind = not(diff(rawdata100segundos(:,7))==0);
rawdata100segundos_noRepeat = rawdata100segundos(ind,:);

acc0 = rawdata100segundos_noRepeat(3:2:end,1:3); % Linha que nao tem nada (MASTER0 1ra tripla)
acc1 = rawdata100segundos_noRepeat(3:2:end,4:6); % Linha com sensor na palma ?? (MASTER0 2da tripla)
acc2 = rawdata100segundos_noRepeat(4:2:end,1:3); % Linha com falange 1 ?? (MASTER1 1ra tripla)
acc3 = rawdata100segundos_noRepeat(4:2:end,4:6); % Linha com falange 2 ?? (MASTER1 2da tripla)

phi_acc0 = acos(acc0(:,3)./(acc0(:,3).^2+acc0(:,2).^2+acc0(:,1).^2).^0.5);
phi_acc1 = acos(acc1(:,3)./(acc1(:,3).^2+acc1(:,2).^2+acc1(:,1).^2).^0.5); % theta_0
phi_acc2 = acos(acc2(:,3)./(acc2(:,3).^2+acc2(:,2).^2+acc2(:,1).^2).^0.5); % theta_1
phi_acc3 = acos(acc3(:,3)./(acc3(:,3).^2+acc3(:,2).^2+acc3(:,1).^2).^0.5); % theta_2
phi_acc1 =  phi_acc1(1:end-1,:);
%% Calcular novos angulos e filtrar usando filtro de media movel
theta_01 = phi_acc2 - phi_acc1;
theta_02 = phi_acc3 - theta_01 - phi_acc1;

window_mean = 100;
theta_01_filt = movmean(theta_01, window_mean);
theta_02_filt = movmean(theta_02, window_mean);

%% DH e vizualizacao
L0 = 30;
a0 = 15;
L = [50, 31, 25]; % Falanges em mm
a =     [0,     L(1),   L(2),   L(3)];
d =     [0,     0,      0,      0];
alpha = [pi/2   0,      0,      0];

mth_base = [rot_axis(-pi/2,2), [0;a0;L0];
            0,0,0,1];

cor = [0 0 0]; espessura = 1.2; larg = 5;
start = 4500;
for i=start:10:length(theta_01_filt)
% Angulos theta de entrada
%      aa    fe_proximal      fe_medial       fe_distal
q = [   0, theta_01_filt(i), theta_02_filt(i), theta_02_filt(i)/2];

A00 = mth_base;
A01 = A00 * denhar(q(1),d(1),a(1),alpha(1));
A02 = A01 * denhar(q(2),d(2),a(2),alpha(2));
A03 = A02 * denhar(q(3),d(3),a(3),alpha(3));
A04 = A03 * denhar(q(4),d(4),a(4),alpha(4));

                     pBase = [0,0,0];
rot0 = A00(1:3,1:3); p0 = A00(1:3,4);
rot1 = A01(1:3,1:3); p1 = A01(1:3,4);
rot2 = A02(1:3,1:3); p2 = A02(1:3,4);
rot3 = A03(1:3,1:3); p3 = A03(1:3,4);
rot4 = A04(1:3,1:3); p4 = A04(1:3,4);

figure(1),clf

subplot(122), hold on
% plot(phi_acc1(start:i)*180/pi)
% plot(phi_acc2(start:i)*180/pi)
% plot(phi_acc3(start:i)*180/pi)
plot(theta_01_filt(start:i)*180/pi)
plot(theta_02_filt(start:i)*180/pi)
grid on, 
% legend
% axis([-inf, inf, -100, 34])

subplot(121), hold on, grid on
% view(-30,30)
view(0,0)
draw_cord_sys(eye(3),pBase,larg,'base');
draw_cord_sys(rot0,p0,larg,'0');
% draw_cord_sys(rot1,p1,larg,'1');
% draw_cord_sys(rot2,p2,larg,'2');
% draw_cord_sys(rot3,p3,larg,'3');
% draw_cord_sys(rot4,p4,larg,'4');
dibujar_linea(pBase,p0,cor,espessura);
dibujar_linea(p1,p2,cor,espessura);
dibujar_linea(p2,p3,cor,espessura);
dibujar_linea(p3,p4,cor,espessura);
xlabel('x'),ylabel('y'),zlabel('z')
axis([-40 80, -25 25 0 140])

pause(0.001);

end