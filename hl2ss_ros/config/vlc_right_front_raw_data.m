function [q_Robot_config, q_camera_config,t_Robot_config,t_camera_config ]=data_quaternion()
% Author: Frank Regal, 2025

%input: no input
%output: Preparing data for the AXXB function based on the collected data
%in Rviz
% robot config is World to HoloLens RigNode

t_Robot_config=[
      0.361, -0.766, -0.321
      0.507, -0.752, -0.319
      0.625, -0.822, -0.320
      0.486, -0.703, -0.323
      0.625, -0.826, -0.320
      0.357, -0.655, -0.325
      ];

q_Robot_config=[
      0.087, 0.097, -0.661, 0.739
      0.104, 0.074, -0.808, 0.575
      -0.095, -0.051, 0.882, -0.458
      0.101, 0.091, -0.732, 0.668
      -0.099, -0.055, 0.875, -0.471
      0.085, 0.105, -0.624, 0.770
    ];

t_camera_config=[
-0.065, -0.088, 0.260
-0.059, -0.099, 0.303
-0.019, -0.088, 0.317
-0.009, -0.114, 0.329
-0.004, -0.090, 0.311
-0.021, -0.122, 0.357
      ];

q_camera_config=[
0.985, 0.013, -0.072, -0.154
0.982, -0.016, 0.120, -0.146
0.957, -0.030, 0.258, -0.130
0.991, -0.005, 0.033, -0.132
0.958, -0.025, 0.241, -0.157
0.971, 0.016, -0.130, -0.202
      ];

