function [action,time_elapsed,explored_actions,explored_nodes]=compute_action(tracks,mcts, alpha_c,alpha_s)
action=[];
time_elapsed=0.0;
explored_actions=[];
explored_nodes=[];
state_means=zeros(length(tracks),6);
%state_covariances=zeros(length(tracks),6,6);
state_covariances=cell(length(tracks),1);
if ~isempty(tracks)
for i=1:length(tracks)
    state_means(i,:)=tracks(i).stateKalmanFilter.State';
    state_covariances{i}=tracks(i).stateKalmanFilter.StateCovariance';
end
[action,time_elapsed,explored_actions,explored_nodes]=get_action(mcts,state_means',state_covariances, alpha_c,alpha_s);
explored_nodes=double(explored_nodes);
%MATLAB!!!
action=action+1;


end
end
