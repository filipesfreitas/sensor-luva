%% Parâmetros fixos
a1 = 3;
a2 = 0;
a3 = 5;
a4 = 3;
a5 = 2.5;
d1=2;
%% Matrizes de transformação homogênea

A0 = D_H_par( 0,-pi/2,0,0);
A1 = D_H_par( 0,alpha1,d1,a1);
A2 = D_H_par( pi/2,alpha2,0,0);
A3 = D_H_par( pi/2,alpha3,0,a3);
A4 = D_H_par( 0,alpha4,0,a4);
A5 = D_H_par( 0,0,0,a5);
A01 = A0*A1;  % matriz de transformação homogênea do frame A0 ao frame A1; Matriz A01
A12 = A01*A2; % matriz de transformação homogênea do frame A1 ao frame A2; Matriz A12
A23 = A12*A3; % matriz de transformação homogênea do frame A2 ao frame A3; Matriz A23
A34 = A23*A4; % matriz de transformação homogênea do frame A3 ao frame A4; Matriz A34
A45 = A34*A5; % matriz de transformação homogênea do frame A4 ao frame A5; Matriz A45

%% Desenhar sistema de coordenadas
clf;
cord_system(A0,'A0');
cord_system(A01,'A1');
cord_system(A12,'A2');
cord_system(A23,'A3');
cord_system(A34,'A4');
cord_system(A45,'A5');

%% Desenhar as linhas que representam os links  entre juntas
p0=A0(1:3,4);
p1=A01(1:3,4);
p2=A12(1:3,4);
p3=A23(1:3,4);
p4=A34(1:3,4);
p5=A45(1:3,4);

%% Desenhar as linhas que representam os links  entre juntas
p0=A0(1:3,4);
p1=A01(1:3,4);
p2=A12(1:3,4);
p3=A23(1:3,4);
p4=A34(1:3,4);
p5=A45(1:3,4);
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

x = [p4(1);p5(1)];
y = [p4(2);p5(2)];
z = [p4(3);p5(3)];
plot3(x,y,z ,'Color', [0 0 0],'LineWidth',3);

