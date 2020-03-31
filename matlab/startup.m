% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

%%%%%%%%%%%
%% Setup %%
%%%%%%%%%%%

% Suppress warning messages
warning('off', 'all');


%%%%%%%%%
%% TRS %%
%%%%%%%%%

% Launch the startup script of TRS
run('trs/matlab/startup_robot.m');

% Add to path all files of trs folder
addpath(genpath('trs/'));
