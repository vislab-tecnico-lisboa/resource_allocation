%load('teste.mat')
mean_average_optimization_times=mean(average_optimization_times,3);
mean_average_detection_times=mean(average_detection_times,3);
mean_average_tracking_times=mean(average_tracking_times,3);
mean_average_total_times=mean_average_optimization_times+mean_average_detection_times+mean_average_tracking_times;
mean_average_mot=mean(average_mot,3);

for c1=1:length(max_items_)
    str=sprintf('max items: %d',max_items_(c1));
    disp(str);
    for c2=1:length(capacity_constraints_)
        str=sprintf(' & %.1f & %.1f & %.1f & %.1f  & %.1f & %.1f & %.1f & %.1f & %.1f & %.1f & %.1f & %.1f & %.1f',...
            mean_average_mot(c1,c2,1,1:3),...
            mean_average_mot(c1,c2,1,5),...
            mean_average_mot(c1,c2,1,7:end),...
            1.0/mean_average_detection_times(c1,c2,1,:));
        disp(str)
    end


end