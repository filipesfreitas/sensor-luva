clear all
clc

xy = csvread('offsetxy');
z = csvread('offsetz');

offsetz = median(z(:,3));
offsety = median(xy(:,2));
offsetx = median(xy(:,1));

offsetgz = median(z(:,6));
offsetgy = median(xy(:,5));
offsetgx = median(xy(:,4));


meanz = mean(z(:,3));
meany = mean(xy(:,2));
meanx = mean(xy(:,1));

meangz = mean(z(:,6));
meangy = mean(xy(:,5));
meangx = mean(xy(:,4));

stdz = std(z(:,3));
stdy = std(xy(:,2));
stdx = std(xy(:,1));

stdgz = std(z(:,6));
stdgy = std(xy(:,5));
stdgx = std(xy(:,4));

xy_test = csvread('offsetxy1');
median(xy_test(:,1))