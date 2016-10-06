close all
load('teste.mat')
mean_average_optimization_times=median(average_optimization_times,3);
mean_average_detection_times=median(average_detection_times,3);
mean_average_tracking_times=median(average_tracking_times,3);
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

%% plots

fontsize=15;
figure(1)

plot(capacity_constraints_,repmat(4.13,length(max_items_)));
hold on
for c1=1:length(max_items_)
plot(capacity_constraints_,1.0./mean_average_detection_times(c1,:,1,:))
hold on
end
set(gca,'XTickLabel',capacity_constraints_)
ylabel('fps','FontSize',fontsize)
xlabel('S_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);


figure(2)
plot(max_items_,repmat(4.13,length(capacity_constraints_)));
hold on
for c2=1:4
plot(max_items_,1.0./mean_average_detection_times(:,c2,1,:))
hold on
end
set(gca,'XTick',1:4)
%set(gca,'XTickLabel',max_items_)
ylabel('fps','FontSize',fontsize)
xlabel('K_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);

motp_index=13;
%MOTPrecision
figure(3)
for c1=1:length(max_items_)
plot(capacity_constraints_,mean_average_mot(c1,:,1,motp_index))
hold on
end
set(gca,'XTickLabel',capacity_constraints_)
ylabel('MOTP','FontSize',fontsize)
xlabel('S_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);

figure(4)
for c2=1:4
plot(max_items_,mean_average_mot(:,c2,1,motp_index))
hold on
end
set(gca,'XTick',1:4)
%set(gca,'XTickLabel',max_items_)
ylabel('MOTP','FontSize',fontsize)
xlabel('K_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);


mota_index=12;
%MOTA
figure(5)
for c1=1:length(max_items_)
plot(capacity_constraints_,mean_average_mot(c1,:,1,mota_index))
hold on
end
set(gca,'XTickLabel',capacity_constraints_)
ylabel('MOTA','FontSize',fontsize)
xlabel('S_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);

figure(6)
for c2=1:4
plot(max_items_,mean_average_mot(:,c2,1,mota_index))
hold on
end
set(gca,'XTick',1:4)
%set(gca,'XTickLabel',max_items_)
ylabel('MOTA','FontSize',fontsize)
xlabel('K_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);

