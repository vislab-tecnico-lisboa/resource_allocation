function mcts_=initializeMCTS(width, height, resource_constraint,max_items,min_width,min_height,max_simulation_time_millis,simulation_depth, action_mode)
mcts_ = mcts(width,height,resource_constraint,max_items,min_width,min_height,max_simulation_time_millis,simulation_depth,action_mode);
% probability_map=get_probability_maps(darap_);
% imagesc(probability_map{1,1});
end