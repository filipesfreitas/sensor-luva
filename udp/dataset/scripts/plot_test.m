%% IEEE Standard Figure Configuration - Version 1.0

% run this code before the plot command

%%
% According to the standard of IEEE Transactions and Journals: 

% Times New Roman is the suggested font in labels. 

% For a singlepart figure, labels should be in 8 to 10 points,
% whereas for a multipart figure, labels should be in 8 points.

% Width: column width: 8.8 cm; page width: 18.1 cm.

%% width & hight of the figure
k_scaling = 4;          % scaling factor of the figure
% (You need to plot a figure which has a width of (8.8 * k_scaling)
% in MATLAB, so that when you paste it into your paper, the width will be
% scalled down to 8.8 cm  which can guarantee a preferred clearness.
k_width_hight = 2;      % width:hight ratio of the figure

width = 8.8 * k_scaling;
hight = width / k_width_hight;

%% figure margins
top = 0.5;  % normalized top margin
bottom = 3;	% normalized bottom margin
left = 3.5;	% normalized left margin
right = 1;  % normalized right margin

%% set default figure configurations
set(0,'defaultFigureUnits','centimeters');
set(0,'defaultFigurePosition',[0 0 width hight]);

set(0,'defaultLineLineWidth',1*k_scaling);
set(0,'defaultAxesLineWidth',0.25*k_scaling);

set(0,'defaultAxesGridLineStyle',':');
set(0,'defaultAxesYGrid','on');
set(0,'defaultAxesXGrid','on');

set(0,'defaultAxesFontName','Times New Roman');
set(0,'defaultAxesFontSize',8*k_scaling);

set(0,'defaultTextFontName','Times New Roman');
set(0,'defaultTextFontSize',8*k_scaling);

set(0,'defaultLegendFontName','Times New Roman');
set(0,'defaultLegendFontSize',8*k_scaling);

set(0,'defaultAxesUnits','normalized');
set(0,'defaultAxesPosition',[left/width bottom/hight (width-left-right)/width  (hight-bottom-top)/hight]);

set(0,'defaultAxesColorOrder',[0 0 0]);
set(0,'defaultAxesTickDir','out');

set(0,'defaultFigurePaperPositionMode','auto');

% you can change the Legend Location to whatever as you wish
set(0,'defaultLegendLocation','bestoutside');
set(0,'defaultLegendBox','on');
set(0,'defaultLegendOrientation','vertical');


test = csvread('imucaracterizacao');
test = test(1:end-1,:);
window = 4;

hold on 
grid on 

%plot(test(:,1)/2048,'r');
plot(test(:,2)/2048,'g');
%plot(test(:,3),'b');
%plot(test(:,4),'m');
%plot(test(:,5),'m');
%plot(test(:,6),'y');
xlabel('amostra n');
ylabel('raw ');
legend('acx','acy','acz','gx','gy','gz');

figure()
hold on 
grid on 


%plot((test(:,1)-79)/2048,'r');
plot((test(:,2)-57)/2048,'g');
%plot(test(:,3),'b');
%plot(test(:,4),'m');
%plot(test(:,5)+1,'m');
%plot(test(:,6)+1,'y');
xlabel('amostra n');
ylabel('raw ');
legend('acx','acy','acz','gx','gy','gz');


offsetx = csvread('offsetxy');



