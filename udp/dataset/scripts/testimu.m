clc
clear all

%format plot

format_plot
% 
metacarpo_phi_measured = [];
metacarpo_theta_measured = [];
proximal_phi_measured = [];
proximal_tehta_measured = [];
x = 0:5:180;

for a=0:5:180
  
  uut = get_degrees('dataset/imudataset/'+string(a)+'graus.csv');
  
  [m s] = medidas(uut.thetametacarpo);
  metacarpo_theta_measured =[metacarpo_theta_measured;m s];
  [m s]  = medidas(uut.phimetacarpo);
  metacarpo_phi_measured = [metacarpo_phi_measured;m s];
  [m s] = medidas(uut.thetaproximal);
  proximal_tehta_measured =[proximal_tehta_measured;m s];
  [m s] = medidas(uut.phiproximal);
  proximal_phi_measured = [proximal_phi_measured;m s];
end

hold on
grid on
grid minor
errorbar(x,metacarpo_theta_measured(:,1),metacarpo_theta_measured(:,2),'b');
errorbar(x,metacarpo_phi_measured(:,1),metacarpo_phi_measured(:,2),'g');
errorbar(x,proximal_tehta_measured(:,1),proximal_tehta_measured(:,2),'r');
errorbar(x,proximal_phi_measured(:,1),proximal_phi_measured(:,2),'m');
legend( 'Metacarpal theta',...
        'Metacarpal phi',...
        'Proximal theta',...
        'Proximal phi')
xlabel('Ângulo configurado (º)')
ylabel('EStimativa da IMU(º)')
xticks([0:20:180])
yticks([0:20:180])

axe=gca
axe.GridColor = 'k';
axe.MinorGridColor='k';
axe.LineWidth = 1
axe.GridAlpha = 0.8

