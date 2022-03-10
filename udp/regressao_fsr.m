clc
clear all
folder = "dataset/fsrdataset/";
patern_filename = 'gramas.csv';
dataset = [];
format_plot
dataset = [dataset, importfsrrawdata(folder + string(35) + patern_filename)];

for i=50:25:500
    dataset = [dataset, importfsrrawdata(folder + string(i) + patern_filename)];
end

x_plot = []

x = [35 50:25:500];

for i=1:1:length(dataset(1,:))
    [m s] = medidas(dataset(:,i));
    x_plot = [x_plot; m s];
end

resistance = 3300 ./ ( 3300 - x_plot(:,1) )*47000;

hold on
grid on
grid minor

errorbar(x,x_plot(:,1),x_plot(:,2),'vertical','*')

xlabel('Peso teste (g)')
ylabel('Tensão Medida (mV)')

y = fit(x',x_plot(:,1),'exp1')

plot(y,x,x_plot(:,1));
xfit = y.a*exp(y.b*x);
legend('Dados experimentais','Curva após regressão')

rmse = sqrt(sum((xfit'-x_plot(:,1)).^2)/17);


