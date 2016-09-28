function observation_matrix=observation_model(state,min_width,min_height)
scales_per_octave=8;
n=8.0*log2(state(3));
q_scale =2.^((n+1)/scales_per_octave)-2.^(n/scales_per_octave);
q_x=min_width*q_scale*0.5;
q_y=min_height*q_scale*0.5;

observation_matrix=[q_x*q_x 0 0; 0 q_y*q_y 0; 0 0 q_scale*q_scale];
end