function [rois,time_elapsed]=compute_action(tracks,mcts)
rois=[];
time_elapsed=0.0;

state_means=zeros(length(tracks),6);
%state_covariances=zeros(length(tracks),6,6);
state_covariances=cell(length(tracks),1);
if ~isempty(tracks)
for i=1:length(tracks)
    state_means(i,:)=tracks(i).stateKalmanFilter.State';
    state_covariances{i}=tracks(i).stateKalmanFilter.StateCovariance;
end
[rois,time_elapsed]=get_action(mcts,state_means',state_covariances);

end
end
