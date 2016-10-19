close all
load('mcts_alpha_0_5.mat')
load('random_0_5_0_5.mat')
load('greedy_0_5_0_5.mat')
load('fullwindow.mat')

% MCTS
mean_average_optimization_times=median(average_optimization_times,3);
mean_average_detection_times=median(average_detection_times,3);
mean_average_tracking_times=median(average_tracking_times,3);
mean_average_total_times=mean_average_optimization_times+mean_average_detection_times+mean_average_tracking_times;
mean_average_mot=mean(average_mot,3);

% RANDOM
mean_average_optimization_times_random=median(average_optimization_times_random,3);
mean_average_detection_times_random=median(average_detection_times_random,3);
mean_average_tracking_times_random=median(average_tracking_times_random,3);
mean_average_total_times_random=mean_average_optimization_times_random+mean_average_detection_times_random+mean_average_tracking_times_random;
mean_average_mot_random=mean(average_mot_random,3);

% GREEDY
mean_average_optimization_times_greedy=median(average_optimization_times_greedy,3);
mean_average_detection_times_greedy=median(average_detection_times_greedy,3);
mean_average_tracking_times_greedy=median(average_tracking_times_greedy,3);
mean_average_total_times_greedy=mean_average_optimization_times_greedy+mean_average_detection_times_greedy+mean_average_tracking_times_greedy;
mean_average_mot_greedy=mean(average_mot_greedy,3);


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

average_detection_time_gains=0.25./average_detection_times(:,:,:);
mean_average_detection_time_gains=mean(average_detection_time_gains(:,:,:),3);
std_average_detection_time_gains=std(average_detection_time_gains(:,:,:),0,3);
fontsize=40;

figure(1)

legend_string={'K_{max}=4',...
    'K_{max}=3',...
    'K_{max}=2',...
    'K_{max}=1',...
    };


for c1=1:length(max_items_)
    errorbar(capacity_constraints_,mean_average_detection_time_gains(c1,:),std_average_detection_time_gains(c1,:))
    hold on
end
xlim([0.09 1.01])
ylabel('Gain','FontSize',fontsize)
xlabel('S_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);
legend(legend_string)

figure(2)

legend_string={'S_{max}=1.0',...
    'S_{max}=0.9',...
    'S_{max}=0.8',...
    'S_{max}=0.7',...
    };

for c2=1:4
    errorbar(max_items_,mean_average_detection_time_gains(:,c2),std_average_detection_time_gains(:,c2))
    hold on
end
set(gca,'XTick',1:4)
xlim([0.9 4.1])

%set(gca,'XTickLabel',max_items_)
ylabel('Gain','FontSize',fontsize)
xlabel('K_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);
legend(legend_string)

%% MOTPrecision

motp_index=13;

mean_average_motp_fullwindow=mean(average_mot_fullwindow(:,motp_index),1);
std_average_motp_fullwindow=std(average_mot_fullwindow(:,motp_index),0,1);

mean_average_motp=mean(average_mot(:,:,:,motp_index),3);
std_average_motp=std(average_mot(:,:,:,motp_index),0,3);

mean_average_motp_random=mean(average_mot_random(:,:,:,motp_index),3);
std_average_motp_random=std(average_mot_random(:,:,:,motp_index),0,3);

mean_average_motp_greedy=mean(average_mot_greedy(:,:,:,motp_index),3);
std_average_motp_greedy=std(average_mot_greedy(:,:,:,motp_index),0,3);

figure(3)

errorbar(capacity_constraints_,...
    repmat(mean_average_motp_fullwindow(:),1,length(capacity_constraints_)),...
    repmat(std_average_motp_fullwindow(:),1,length(capacity_constraints_)),'k--');

hold on
for c1=1:length(max_items_)
    errorbar(capacity_constraints_,mean_average_motp(c1,:),std_average_motp(c1,:))
    errorbar(capacity_constraints_,mean_average_motp_random(c1,:),std_average_motp_random(c1,:),'--')
    errorbar(capacity_constraints_,mean_average_motp_greedy(c1,:),std_average_motp_greedy(c1,:),'x')

    hold on
end
xlim([0.09 1.01])
ylabel('MOTP','FontSize',fontsize)
xlabel('S_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);

figure(4)

errorbar(max_items_,...
    repmat(mean_average_motp_fullwindow(:),1,length(max_items_)),...
    repmat(std_average_motp_fullwindow(:),1,length(max_items_)),'k--');

hold on

for c2=1:4
    errorbar(max_items_,mean_average_motp(:,c2),std_average_motp(:,c2))
    errorbar(max_items_,mean_average_motp_random(:,c2),std_average_motp_random(:,c2),'--')
    errorbar(max_items_,mean_average_motp_greedy(:,c2),std_average_motp_greedy(:,c2),'x')

    hold on
end
set(gca,'XTick',1:4)
xlim([0.9 4.1])
%set(gca,'XTickLabel',max_items_)
ylabel('MOTP','FontSize',fontsize)
xlabel('K_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);

%% false positives
false_positives_index=8;
mean_average_fp_fullwindow=mean(average_mot_fullwindow(:,false_positives_index),1);
std_average_fp_fullwindow=std(average_mot_fullwindow(:,false_positives_index),0,1);


mean_average_fp=mean(average_mot(:,:,:,false_positives_index),3);
std_average_fp=std(average_mot(:,:,:,false_positives_index),0,3);

figure(5)

errorbar(capacity_constraints_,...
    repmat(mean_average_fp_fullwindow(:),1,length(capacity_constraints_)),...
    repmat(std_average_fp_fullwindow(:),1,length(capacity_constraints_)),'k--');

hold on

for c1=1:length(max_items_)
    errorbar(capacity_constraints_,mean_average_fp(c1,:),mean_average_fp(c1,:))
    hold on
end
xlim([0.09 1.01])
ylabel('FP','FontSize',fontsize)
xlabel('S_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);

figure(6)

errorbar(max_items_,...
    repmat(mean_average_fp_fullwindow(:),1,length(max_items_)),...
    repmat(std_average_fp_fullwindow(:),1,length(max_items_)),'k--');

hold on

for c2=1:4
    errorbar(max_items_,mean_average_fp(:,c2),mean_average_fp(:,c2))
    hold on
end
set(gca,'XTick',1:4)
xlim([0.9 4.1])
%set(gca,'XTickLabel',max_items_)
ylabel('FP','FontSize',fontsize)
xlabel('K_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);


%% false negatives
false_negatives_index=9;

mean_average_fn_fullwindow=mean(average_mot_fullwindow(:,false_negatives_index),1);
std_average_fn_fullwindow=std(average_mot_fullwindow(:,false_negatives_index),0,1);

mean_average_fn=mean(average_mot(:,:,:,false_negatives_index),3);
std_average_fn=std(average_mot(:,:,:,false_negatives_index),0,3);

figure(7)

errorbar(capacity_constraints_,...
    repmat(mean_average_fn_fullwindow(:),1,length(capacity_constraints_)),...
    repmat(mean_average_fn_fullwindow(:),1,length(capacity_constraints_)),'k--');

hold on

for c1=1:length(max_items_)
    errorbar(capacity_constraints_,mean_average_fn(c1,:),mean_average_fn(c1,:))
    hold on
end
xlim([0.09 1.01])
ylabel('FN','FontSize',fontsize)
xlabel('S_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);

figure(8)

errorbar(max_items_,...
    repmat(mean_average_fn_fullwindow(:),1,length(max_items_)),...
    repmat(mean_average_fn_fullwindow(:),1,length(max_items_)),'k--');

hold on

for c2=1:4
    errorbar(max_items_,mean_average_fn(:,c2),mean_average_fn(:,c2))
    hold on
end
set(gca,'XTick',1:4)
xlim([0.9 4.1])
%set(gca,'XTickLabel',max_items_)
ylabel('FN','FontSize',fontsize)
xlabel('K_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);

%% MOTA
mota_index=14;

mean_average_mota_fullwindow=mean(average_mot_fullwindow(:,mota_index),1);
std_average_mota_fullwindow=std(average_mot_fullwindow(:,mota_index),0,1);

mean_average_mota=mean(average_mot(:,:,:,mota_index),3);
std_average_mota=std(average_mot(:,:,:,mota_index),0,3);

figure(9)

errorbar(capacity_constraints_,...
    repmat(mean_average_mota_fullwindow(:),1,length(capacity_constraints_)),...
    repmat(std_average_mota_fullwindow(:),1,length(capacity_constraints_)),'k--');

hold on

for c1=1:length(max_items_)
    errorbar(capacity_constraints_,mean_average_mota(c1,:),std_average_mota(c1,:))
    hold on
end
xlim([0.09 1.01])
ylabel('MOTA','FontSize',fontsize)
xlabel('S_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);

figure(10)

errorbar(max_items_,...
    repmat(mean_average_mota_fullwindow(:),1,length(max_items_)),...
    repmat(mean_average_mota_fullwindow(:),1,length(max_items_)),'k--');

hold on

for c2=1:4
    errorbar(max_items_,mean_average_mota(:,c2),std_average_mota(:,c2))
    hold on
end
set(gca,'XTick',1:4)
xlim([0.9 4.1])
%set(gca,'XTickLabel',max_items_)
ylabel('MOTA','FontSize',fontsize)
xlabel('K_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);


%% ID switches
ids_index=10;

mean_average_ids_fullwindow=mean(average_mot_fullwindow(:,ids_index),1);
std_average_ids_fullwindow=std(average_mot_fullwindow(:,ids_index),0,1);

mean_average_ids=mean(average_mot(:,:,:,ids_index),3);
std_average_ids=std(average_mot(:,:,:,ids_index),0,3);

figure(11)

errorbar(capacity_constraints_,...
    repmat(mean_average_ids_fullwindow(:),1,length(capacity_constraints_)),...
    repmat(std_average_ids_fullwindow(:),1,length(capacity_constraints_)),'k--');

hold on

for c1=1:length(max_items_)
    errorbar(capacity_constraints_,mean_average_ids(c1,:),std_average_ids(c1,:))
    hold on
end
xlim([0.09 1.01])
ylabel('IDs','FontSize',fontsize)
xlabel('S_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);

figure(12)

errorbar(max_items_,...
    repmat(mean_average_ids_fullwindow(:),1,length(max_items_)),...
    repmat(std_average_ids_fullwindow(:),1,length(max_items_)),'k--');

hold on

for c2=1:4
    errorbar(max_items_,mean_average_ids(:,c2),std_average_ids(:,c2))
    hold on
end
set(gca,'XTick',1:4)
xlim([0.9 4.1])
%set(gca,'XTickLabel',max_items_)
ylabel('IDs','FontSize',fontsize)
xlabel('K_{max}','FontSize',fontsize)
set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);


