function [] = plot_var(input,window,title_input)


figure;
hold on;
plot(input,'r');
plot(ones(length(input),1)*mean(input),'b');
plot(movmean(input,window),'k');
plot(movavg(input,'exponential',window),'g');
title(title_input);
xlabel("sample (50 ms)");
ylabel("Angle (°)");
legend("raw data","mean","mov mean","mov mean expo");
hold off;
set(gca,'XMinorTick','on','YMinorTick','on');
end

