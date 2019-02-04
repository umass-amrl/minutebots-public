% Reads log files and trains full linear and scalar motion models for the
% minutebots 


clear variables;
close all;
clc;

%% Parameters

Robots = {'00', '01', '02', '03', '04', '05', '06', '07', '08'};
Trajectory_Types = {'000','001', '002', '003', '004', '005', '006', '007', '008', '009', '010', '011', '012'};

default_inv_model_wh_based =  [24.6744  -19.9809   -2.5084;...
                               22.4506   22.4506   -2.5084;...
                              -22.4506   22.4506   -2.5084;...
                              -24.6744  -19.9809   -2.5084];

% ********************************************************************************
% ******** MODIFY THE FOLLOWING PARAMETERS ACCORDING TO TRAINING PREFERENCES *****
% ********************************************************************************

% Set to true to save the trained models to file
SAVE_MODEL = true;

% The path that will be used for saving trained models
% MODELS_PATH = '/home/srabiee/SSL/model_learning/learned_models/';
MODELS_PATH = '../../model_learning/learned_models/';

% The path to log files that will be used for training
% LOGS_PATH = '../logs/cmd_obs/*.csv';
LOGS_PATH = '../../model_learning/logs/cmd_obs/';

% Set to true to turn on visualizations
VISUALIZATION_ON = false;

% Set to true to sample training points uniformly within the robot velocity
% bounds
UNIFORM_SAMPLING = false;

% Defines the number of training data that will be uniformly sampled and
% used for training.
TRAIN_DATA_FRACTION = 1/5;

% List of robots, for which models will be trained                   
robots_to_analyze = 0:8; % robot 04       

% List of trajectory numbers, logs of which will be used for training.
% Trajectory 0 means logs of joystick driving the robots
trajectories_to_search = [0]; 
% trajectories_to_search = [1, 2, 3, 4, 5, 9, 10, 11, 12]; 

% Choose the list of log file numbers that you want to use for model learning                         
% lookup_list = 1:length(fileNames);
% lookup_list = 1:21; % includes the manual driving log files that the model was initially trained on
% lookup_list = 22:27; % manual driving to test the trained model: robot#4 (Fall_2017): floor int conversion
% lookup_list = 28:32; % manual driving to test the trained model: robot#4 (May-17-2018): floor int conversion
% lookup_list = 33:38; % manual driving to test the trained model: robot#4 (May-20-2018): static_cast conversion
% lookup_list = 33:34;

lookup_list = 37:47; % manual driving for robots 1,2,3,4,6,7 (May-22-2018)                          

% Define a list of bag file numbers that you want to exclude from the
% lookup_list
omitted_list = [];

% Velocity limits for the robots. The limits are used for uniformly
% sampling training data points within the bounds
MAX_LIN_VEL =  4; % m/s
MAX_ANG_VEL = 4 * pi; %rad/s

 
%% Loading Data
fileNames = dir([LOGS_PATH, '*.csv']);
trajectory_specific_files = cell(length(Trajectory_Types), 1);
robot_specific_files = cell(length(Robots), 1);

% Lookup all trajectory numbers
for i = 1: length(fileNames)
   file_name = fileNames(i).name;
   log_file_num = str2num(file_name(1:5));
   
   
   traj_num = str2num(file_name(7:9));
   traj_num =traj_num + 1;
   robot_name = file_name(11:12);
   
   trajectory_specific_files{traj_num} = [trajectory_specific_files{traj_num}; log_file_num];
   
   robot_index = find(strcmp(Robots, robot_name));
   robot_specific_files{robot_index} = [robot_specific_files{robot_index}; log_file_num];
   
end

%% Processing 
% Convert the robot and trajectory numbers to their corresponding indices
robots_to_analyze = robots_to_analyze + 1;
trajectories_to_search = trajectories_to_search + 1;

if(~UNIFORM_SAMPLING)
   warning('Uniform sampling is turned off. All data will be used for training.')
end

counter_main = 0;
for kk = reshape(robots_to_analyze, 1, length(robots_to_analyze))
    counter_main = counter_main + 1;
    fprintf('***********************\n');
    fprintf('Robot#%s\n', Robots{kk});
    fprintf('***********************\n');
    
    % Put together bagfiles from all trajectories of interest
    chosen_trajectory_files = [];
    for i =  1 : length(trajectories_to_search)
        chosen_trajectory_files = [chosen_trajectory_files; trajectory_specific_files{trajectories_to_search(i)}];
    end
    
    complete_list = intersect(chosen_trajectory_files, robot_specific_files{kk});
    complete_list = intersect(complete_list, lookup_list);
    pruned_list = setdiff(complete_list, omitted_list);
    final_list = pruned_list;
    
    Vision_ang_vel_list_global = [];
    Vision_lin_vel_list_global = [];
    Vision_lin_vel_x_local_list_global = [];
    Vision_lin_vel_y_local_list_global = [];
    
    cmd_wh_vel_list_global = [];

    cmd_ang_vel_list_global = [];
    cmd_lin_vel_list_global = [];
    cmd_lin_vel_x_list_global = [];
    cmd_lin_vel_y_list_global = [];
    
    if(isempty(final_list))
       fprintf("No training data was found for robot# %s!\n",Robots{kk}); 
       continue;
    end

    counter = 0;
    for m = reshape(final_list, 1, length(final_list))
        counter = counter + 1; 
        
        prefix = sprintf('%05d', m);
        selected_file_name = dir([LOGS_PATH,prefix,'*.csv']);
        data = load(fullfile(selected_file_name.folder ,selected_file_name.name));
        
        if(isempty(data))
            fprintf('Logfile# %d is empty!\n', m);
            continue;
        end
        
        data_Vision = data(:, 5:8);
        data_cmd = data(:,1:4);
        
        % find out which trajectory is the current log file for
        current_traj_index = 0;
        for i =  1 : length(trajectories_to_search)
            traj_log_files = trajectory_specific_files{trajectories_to_search(i)};
            if (any(m == traj_log_files))
                current_traj_index = i;
                break;
            end
        end
        
        if (current_traj_index == 0)
            error('Could not find the trajectory number for current log file');
        end
        
        % filter out data points were recorded time equals zero
        pruned_indices_vision = data_Vision(:,1) ~= 0;
        pruned_indices_cmd = data_cmd(:,1) ~= 0;
        pruned_indices = and(pruned_indices_vision, pruned_indices_cmd);
        data_Vision = data_Vision(pruned_indices, :);
        data_cmd = data_cmd(pruned_indices, :);
        

        % Parsing the visual odometry data
        Vision_time = data_Vision(:,1);
        Vision_del_t = diff(Vision_time);
        Vision_pose = data_Vision(:, 2:4);
        Vision_pose(:, 1:2) = Vision_pose(:, 1:2) / 1000.0; % convert mm/s to m/s
        Vision_robot_theta = Vision_pose(:,3);


        % Parsing Command velocity data
        cmd_time = data_cmd(:,1);
        cmd_del_t = diff(cmd_time);
        cmd_linear_vel = data_cmd(:, 2:3) / 1000.0;  % convert mm/s to m/s
        cmd_ang_vel = data_cmd(:, 4);
        
        %% Convert the command velocities to the commanded wheel velocities
        wheel_angles = deg2rad([90 + 51, 90 + 135, 90 - 135, 90 - 51]);
        wheel_directions = [1, 1, 1, 1];
        
        wheel_vel = zeros(length(cmd_time), 4);
        
        commanded_x = cmd_linear_vel(:, 1)  * (127 / 4);
        commanded_y = cmd_linear_vel(:, 2)  * (127 / 4);
        commanded_r = cmd_ang_vel * (127 / 50.63);
        
        % The original with flooring
%         for wh_i = 1:4
%             wheel_vel(:, wh_i) = floor(commanded_x .* cos(wheel_angles(wh_i)) + ...
%                                        commanded_y .* sin(wheel_angles(wh_i)) + ...
%                                        commanded_r) * -wheel_directions(wh_i);
%             
%         end

%         The new version with static_cast method of conversion to integers
        for wh_i = 1:4  
            signs = sign(commanded_x .* cos(wheel_angles(wh_i)) + ...
                                       commanded_y .* sin(wheel_angles(wh_i)) + ...
                                       commanded_r);
                                   
            wheel_vel(:, wh_i) = signs .* floor(abs(commanded_x .* cos(wheel_angles(wh_i)) + ...
                                       commanded_y .* sin(wheel_angles(wh_i)) + ...
                                       commanded_r)) * -wheel_directions(wh_i);
        end

        % Without conversion to integers
%         for wh_i = 1:4
%             
%             signs = sign(commanded_x .* cos(wheel_angles(wh_i)) + ...
%                                        commanded_y .* sin(wheel_angles(wh_i)) + ...
%                                        commanded_r);
%                                    
%             wheel_vel(:, wh_i) = signs .* abs(commanded_x .* cos(wheel_angles(wh_i)) + ...
%                                        commanded_y .* sin(wheel_angles(wh_i)) + ...
%                                        commanded_r) * -wheel_directions(wh_i);
%             
%         end
        

        %% Calculate the desired full path given only the velocity commands
            % [x, y, theta]
        pose_cmd = zeros(length(data_cmd), 3);
        pose_cmd(1,:) = Vision_pose(1,:);

        % *****************************
        % Estimate the pose_cmd of the robot based on only the command velocities
        delta_thetas = cmd_ang_vel(1:end - 1) .* cmd_del_t;
        pose_cmd(2:end, 3) = delta_thetas;
        pose_cmd(:,3) = cumsum(pose_cmd(:,3));
        thetas = pose_cmd(:,3);

        delta_xs = cmd_linear_vel(1:end - 1, 1) .* cmd_del_t .* cos(thetas(1:end -1)) - cmd_linear_vel(1:end - 1, 2) .* cmd_del_t .* sin(thetas(1:end -1));
        delta_ys = cmd_linear_vel(1:end - 1, 1) .* cmd_del_t .* sin(thetas(1:end -1)) + cmd_linear_vel(1:end - 1, 2) .* cmd_del_t .* cos(thetas(1:end -1));

        pose_cmd(2:end, 1) = delta_xs;
        pose_cmd(2:end, 2) = delta_ys;
        pose_cmd(:,1) = cumsum(pose_cmd(:,1));
        pose_cmd(:,2) = cumsum(pose_cmd(:,2));

        %% Synchronize Commanded Velocity and Measured Velocity 
               
%         StartTrimNum_Train = 0;
%         EndTrimNum_Train = 0;
        StartTrimNum_Train = 15;
        EndTrimNum_Train = 15;
%         StartTrimNum_Train = 60;
%         EndTrimNum_Train = 60;
%         Median_Filter_Window = 10;
        Median_Filter_Window = 10;

        % Calculate the ground truth angular and linear velocity
        
        wraped_diff_ang = wrapToPi(diff(Vision_robot_theta));
        Vision_ang_vel_robot =  wraped_diff_ang./ Vision_del_t;
        Vision_ang_vel_robot = medfilt1(Vision_ang_vel_robot, Median_Filter_Window);
        
        % Linear velocity in global reference frame
        Vision_lin_del_pos = Vision_pose(2:end, 1:2) - Vision_pose(1:end-1, 1:2);
        Vision_lin_vel = Vision_lin_del_pos ./ repmat(Vision_del_t, 1,2); % n * 2 [vx, vy] global vx and vy

        % Linear velocity in local reference frame
        Vision_vel_x_local = Vision_lin_vel(:,1) .* cos(Vision_robot_theta(2:end)) + Vision_lin_vel(:,2) .* sin(Vision_robot_theta(2:end));
        Vision_vel_y_local = - Vision_lin_vel(:,1) .* sin(Vision_robot_theta(2:end)) + Vision_lin_vel(:,2) .* cos(Vision_robot_theta(2:end));
        Vision_vel_x_local = medfilt1(Vision_vel_x_local, Median_Filter_Window);
        Vision_vel_y_local = medfilt1(Vision_vel_y_local, Median_Filter_Window);

        Vision_lin_vel_abs = sqrt(sum(Vision_lin_vel.^2, 2));
        Vision_lin_vel_abs = medfilt1(Vision_lin_vel_abs, Median_Filter_Window);

        % Set up a system of equations between ground truth and commanded
        % velocity values (match those that correspond to the same time instance)
        cmd_lin_vel_list = [];
        cmd_wh_vel_list = [];
        VO_lin_vel_list = [];
        
        cmd_lin_vel_x_list = [];
        Vision_lin_vel_x_local_list = [];
        
        cmd_lin_vel_y_list = [];
        Vision_lin_vel_y_local_list = [];

        cmd_ang_vel_list = [];
        VO_ang_vel_list = [];
        
        
        
        instance_num = length(Vision_time) - EndTrimNum_Train - StartTrimNum_Train;

        % Find the closest commands to the VO time stamps
        instance_counter = 0;
        time_diff_threshold = 1/30; %second
        for k = StartTrimNum_Train + 1:instance_num + StartTrimNum_Train
            
            current_Vision_time = Vision_time(k);            
            current_time_ind_cmd = find(cmd_time <= current_Vision_time, 1, 'last');
            
            if (abs(current_Vision_time - cmd_time(current_time_ind_cmd)) > time_diff_threshold)
                continue;
            end
            
            % continue if no corresponding command vel has been found
            if (isempty(current_time_ind_cmd))
                continue;
            end
            
            instance_counter = instance_counter + 1;

            cmd_wh_vel_list(instance_counter, :) = wheel_vel(current_time_ind_cmd, :);
            
            cmd_ang_vel_list(instance_counter) = cmd_ang_vel(current_time_ind_cmd);
            cmd_lin_vel_list(instance_counter) = sqrt(cmd_linear_vel(current_time_ind_cmd, 1)^2 + cmd_linear_vel(current_time_ind_cmd, 2)^2);
            cmd_lin_vel_x_list(instance_counter) = cmd_linear_vel(current_time_ind_cmd, 1);
            cmd_lin_vel_y_list(instance_counter) = cmd_linear_vel(current_time_ind_cmd, 2);

            VO_ang_vel_list(instance_counter) = Vision_ang_vel_robot(k);
            VO_lin_vel_list(instance_counter) = Vision_lin_vel_abs(k);
            Vision_lin_vel_x_local_list(instance_counter) = Vision_vel_x_local(k);
            Vision_lin_vel_y_local_list(instance_counter) = Vision_vel_y_local(k);
            

        end
        
        cmd_ang_vel_list = cmd_ang_vel_list';
        cmd_lin_vel_list = cmd_lin_vel_list';
        cmd_lin_vel_x_list = cmd_lin_vel_x_list';
        cmd_lin_vel_y_list = cmd_lin_vel_y_list';
        VO_ang_vel_list = VO_ang_vel_list';
        VO_lin_vel_list = VO_lin_vel_list';
        Vision_lin_vel_x_local_list = Vision_lin_vel_x_local_list';
        Vision_lin_vel_y_local_list = Vision_lin_vel_y_local_list';
%      -------------------------------------------------------------------

        %% Filter Training Data
%         indices_of_interest_cmd = find(abs(cmd_lin_vel_list) > 50);
%         indices_of_interest_cmd = find(abs(cmd_lin_vel_list) > 500 & abs(cmd_ang_vel_list) > 0.1);
%         indices_of_interest_cmd = find(abs(cmd_lin_vel_list) > 0.1 & abs(cmd_lin_vel_list) < 1);
        indices_of_interest_cmd = 1 : size(cmd_lin_vel_list, 1);

        cmd_wh_vel_list = cmd_wh_vel_list(indices_of_interest_cmd, :);

        cmd_ang_vel_list = cmd_ang_vel_list(indices_of_interest_cmd);
        cmd_lin_vel_list = cmd_lin_vel_list(indices_of_interest_cmd);
        cmd_lin_vel_x_list = cmd_lin_vel_x_list(indices_of_interest_cmd);
        cmd_lin_vel_y_list = cmd_lin_vel_y_list(indices_of_interest_cmd);
        VO_ang_vel_list = VO_ang_vel_list(indices_of_interest_cmd);
        VO_lin_vel_list = VO_lin_vel_list(indices_of_interest_cmd);
        Vision_lin_vel_x_local_list = Vision_lin_vel_x_local_list(indices_of_interest_cmd);
        Vision_lin_vel_y_local_list = Vision_lin_vel_y_local_list(indices_of_interest_cmd);
        
        % Since the velocity values are raw velocities given the SSL
        % vision data, there exist NAN values, which we should remove:
        non_nan_ind_lin = find(~isnan(VO_lin_vel_list));
        non_nan_ind_ang = find(~isnan(VO_ang_vel_list));
        non_nan_ind = union(non_nan_ind_lin, non_nan_ind_ang);
        VO_lin_vel_list = VO_lin_vel_list(non_nan_ind);
        VO_ang_vel_list = VO_ang_vel_list(non_nan_ind);
        Vision_lin_vel_x_local_list = Vision_lin_vel_x_local_list(non_nan_ind);
        Vision_lin_vel_y_local_list = Vision_lin_vel_y_local_list(non_nan_ind);
        
        cmd_wh_vel_list = cmd_wh_vel_list(non_nan_ind, :);
        cmd_ang_vel_list = cmd_ang_vel_list(non_nan_ind);
        cmd_lin_vel_list = cmd_lin_vel_list(non_nan_ind);
        cmd_lin_vel_x_list = cmd_lin_vel_x_list(non_nan_ind);
        cmd_lin_vel_y_list = cmd_lin_vel_y_list(non_nan_ind);
        
        Vision_ang_vel_list_global = [Vision_ang_vel_list_global; VO_ang_vel_list];
        Vision_lin_vel_list_global = [Vision_lin_vel_list_global; VO_lin_vel_list];
        Vision_lin_vel_x_local_list_global = [Vision_lin_vel_x_local_list_global; Vision_lin_vel_x_local_list];
        Vision_lin_vel_y_local_list_global = [Vision_lin_vel_y_local_list_global; Vision_lin_vel_y_local_list];

        cmd_lin_vel_list_global = [cmd_lin_vel_list_global; cmd_lin_vel_list];
        cmd_lin_vel_x_list_global = [cmd_lin_vel_x_list_global; cmd_lin_vel_x_list];
        cmd_lin_vel_y_list_global = [cmd_lin_vel_y_list_global; cmd_lin_vel_y_list];
        cmd_ang_vel_list_global = [cmd_ang_vel_list_global; cmd_ang_vel_list];
        
        cmd_wh_vel_list_global = [cmd_wh_vel_list_global; cmd_wh_vel_list];
         
        % Visualize the commanded and the observed trajectory
        if (VISUALIZATION_ON)
            figure
            subplot(2,1,1)
            plot(pose_cmd(:,1), pose_cmd(:,2));
            xlabel('x (m)');
            ylabel('y (m)');
            axis equal
            title(['Commanded Path: log file #', num2str(m)]);
            hold off

            subplot(2,1,2)
            plot(Vision_pose(5:end,1), Vision_pose(5:end,2));
            xlabel('x (m)');
            ylabel('y (m)');
            axis equal
            title(['Observed Path: log file #', num2str(m)]);
            hold off
        end
    end

    actual_vel_global =[Vision_lin_vel_x_local_list_global'; Vision_lin_vel_y_local_list_global'; Vision_ang_vel_list_global'];
    cmd_vel_list_global = [cmd_lin_vel_x_list_global, cmd_lin_vel_y_list_global, cmd_ang_vel_list_global];

    if(isempty(cmd_vel_list_global))
       fprintf("No training data was found for robot# %s!\n",Robots{kk}); 
       continue;
    end

    %% Sample from the training data
    if (UNIFORM_SAMPLING)
        TRAINING_DATA_SIZE = floor(size(cmd_wh_vel_list_global, 1) * TRAIN_DATA_FRACTION);
        % TRAINING_DATA_SIZE = 2000;
        % TRAINING_DATA_SIZE = 3000;

        MIN_LIN_VEL = -MAX_LIN_VEL;
        MIN_ANG_VEL = - MAX_ANG_VEL;
        MIN_DISTANCE_SAMPLING = 0.5;

        max_vel_list = [MAX_LIN_VEL, MAX_LIN_VEL, MAX_ANG_VEL];
        min_vel_list = -max_vel_list;

        % uniform sampling
        fprintf("Sampling training data...\n");
        % Sample from the above data points such that you end up with a uniform
        % dataset in the input wheel velocity [w_1, w_2, w_3, w_4] space
        actual_vel_uniform_samp = zeros(TRAINING_DATA_SIZE, 3); % [vx, vy, theta_dot]
        cmd_vel_uniform_samp = zeros(TRAINING_DATA_SIZE, 3);% [vx, vy, theta_dot]
        cmd_wh_vel_uniform_samp =  zeros(TRAINING_DATA_SIZE, 4); % [wh1, wh2, wh3, wh4]

        counter_sampling = 0;
        while (counter_sampling < TRAINING_DATA_SIZE)
            % sample a cmd velocity 
            cmd_vel_sample = (rand(1,3) .* (max_vel_list - min_vel_list)) + min_vel_list;

            % Find the closes point in actual commanded velocities to the sample
            % of interest
            [k,d] = dsearchn(cmd_vel_list_global,cmd_vel_sample);

            if (d <= MIN_DISTANCE_SAMPLING)
                counter_sampling = counter_sampling + 1;
                cmd_wh_vel_uniform_samp(counter_sampling, :) = cmd_wh_vel_list_global(k, :);
                cmd_vel_uniform_samp(counter_sampling, :) = cmd_vel_list_global(k, :);
                actual_vel_uniform_samp(counter_sampling, :) = actual_vel_global(:, k)';
            end
        end

        if (VISUALIZATION_ON)
            figure
            scatter3(cmd_vel_list_global(:,1), cmd_vel_list_global(:,2), cmd_vel_list_global(:,3))
            hold on
            scatter3(cmd_vel_uniform_samp(:,1), cmd_vel_uniform_samp(:,2), cmd_vel_uniform_samp(:,3),'MarkerEdgeColor', 'red' );
            xlabel('lin-vel-x (m/s)');
            ylabel('lin-vel-y (m/s)');
            zlabel('ang-vel (rad/s)');
            title('Sampled Training data');
        end
    else
        cmd_wh_vel_uniform_samp = cmd_wh_vel_list_global;
        cmd_vel_uniform_samp = cmd_vel_list_global;
        actual_vel_uniform_samp = actual_vel_global';
    end

    %% Training Models on the whole data
    
    
    actual_vel_global =[Vision_lin_vel_x_local_list_global'; Vision_lin_vel_y_local_list_global'; Vision_ang_vel_list_global'];
    actual_vel_global_g = actual_vel_global;
    lin_model_params_wh_global = actual_vel_global / cmd_wh_vel_list_global';
    disp('Linear Model with wheel velocities as input (unbalanced training data)');
    lin_model_params_wh_global

    % Train based on uniformly sampled training data
    lin_model_params_uniform_samp_wh_global = actual_vel_uniform_samp' / cmd_wh_vel_uniform_samp';
    lin_inv_model_params_uniform_samp_wh_global = cmd_wh_vel_uniform_samp' / actual_vel_uniform_samp';
    disp('Linear Model with wheel velocities as input (uniformly sampled training data)');
    lin_model_params_uniform_samp_wh_global
    disp('Linear Invserse Model with desired velocities as input and wheel velocity as output (uniformly sampled training data)');
    lin_inv_model_params_uniform_samp_wh_global


    % Scalar model (independent scalar values for linear and angular velocities)
    actual_lin_vel_abs = sqrt(actual_vel_global_g(1,:).^2 + actual_vel_global_g(2,:).^2);
    lin_vel_scale = actual_lin_vel_abs / cmd_lin_vel_list_global';
    ang_vel_scale = actual_vel_global_g(3,:) / cmd_ang_vel_list_global';
    scalar_inv_model_wh_based = default_inv_model_wh_based;
    scalar_inv_model_wh_based(:, 1:2) = scalar_inv_model_wh_based(:, 1:2) / lin_vel_scale;
    scalar_inv_model_wh_based(:, 3) = scalar_inv_model_wh_based(:, 3) / ang_vel_scale;

    % The ideal model that transforms wheel velocities to the robot velocity
    % Use this only when you choose the conversion from command velocities
    % to wheel velocities that does not convert values to integers. The
    % idea of having this ideal model is to calculate the equivalent matrix
    % version of the geometric kinematic model that is being used for
    % conversion from the robot command velocities to the wheel velocities
    % and the other way around.
    ideal_model_wh_based = cmd_vel_list_global'/ cmd_wh_vel_list_global';
    ideal_inv_model_wh_based = cmd_wh_vel_list_global'/ cmd_vel_list_global';


    %% Evaluating Trained Models

    % Calculate estimated angular velocity by the linear model
    estimated_actual_vel_full_linear = lin_model_params_uniform_samp_wh_global * cmd_wh_vel_list_global';

    % Calculate estimated angular velocity by the scalar model
    estimated_x_vel = lin_vel_scale * cmd_lin_vel_x_list_global;
    estimated_y_vel = lin_vel_scale * cmd_lin_vel_y_list_global;
    estimated_ang_vel = ang_vel_scale * cmd_ang_vel_list_global;
    estimated_actual_vel_scalar_model = [estimated_x_vel'; estimated_y_vel'; estimated_ang_vel'];

    % Choose the model, for which you want to visualize the estimated
    % velocity vs. the commanded and observed velocities
%     estimated_actual_vel = estimated_actual_vel_full_linear;
    estimated_actual_vel = estimated_actual_vel_scalar_model;

    %% Save Model
    if (SAVE_MODEL)   
        current_robot = robots_to_analyze(kk);

        prefix = sprintf('%02d', current_robot);
        dlmwrite([MODELS_PATH, prefix, '_inverse_model.csv' ], lin_inv_model_params_uniform_samp_wh_global, 'precision', '%.10f');
        dlmwrite([MODELS_PATH, prefix, '_forward_model.csv' ], lin_model_params_uniform_samp_wh_global, 'precision', '%.10f');
        dlmwrite([MODELS_PATH, prefix, '_scalar_inverse_model.csv' ], scalar_inv_model_wh_based, 'precision', '%.10f');
    end

    %% Visualization
    if (VISUALIZATION_ON)
        % Visualize the matched commanded and measured velocities
        figure
        subplot(3,1,1)
        plot(Vision_lin_vel_x_local_list_global, 'b');
        hold on
        plot(cmd_lin_vel_x_list_global, 'r', 'LineWidth', 2);
        hold on
        plot(estimated_actual_vel(1,:), 'g');
        title('X Velocity (robot ref frame)');
        legend({'measured', 'commanded', 'estimated'});
        ylabel('m/s');
        xlabel('index');

        subplot(3,1,2)
        plot(Vision_lin_vel_y_local_list_global, 'b');
        hold on
        plot(cmd_lin_vel_y_list_global, 'r', 'LineWidth', 2);
        hold on
        plot(estimated_actual_vel(2,:), 'g');
        title('Y Velocity (robot ref frame)');
        legend({'measured', 'commanded', 'estimated'});
        ylabel('m/s');
        xlabel('index');

        subplot(3,1,3)
        plot(Vision_ang_vel_list_global, 'b');
        hold on
        plot(cmd_ang_vel_list_global, 'r', 'LineWidth', 2)
        hold on
        plot(estimated_actual_vel(3,:), 'g');
        title('ang Velocity');
        legend({'measured', 'commanded', 'estimated'});
        ylabel('rad/s');
        xlabel('index');




        % Scatter plot of all commanded velocity values
        figure
        scatter(cmd_lin_vel_list_global, cmd_ang_vel_list_global);
        xlabel('Linear velocity (m/s)');
        ylabel('Angualr velocity (rad/s)');
        title('Command Velocites');

        % Scatter plot of all measured velocity values 
        figure
        scatter(Vision_lin_vel_list_global, Vision_ang_vel_list_global);
        xlabel('Linear velocity (m/s)');
        ylabel('Angualr velocity (rad/s)');
        title('Measured Velocities');

        % Scatter plot of all local vx and vy's
        figure
        scatter(cmd_lin_vel_x_list_global, cmd_lin_vel_y_list_global);
        xlabel('X velocity (m/s)');
        ylabel('Y velocity (m/s)');
        title('Command Velocites (Linear Velocities)');

        % Scatter plot of all measured velocity values 
        figure
        scatter(Vision_lin_vel_x_local_list_global, Vision_lin_vel_y_local_list_global);
        xlabel('X velocity (m/s)');
        ylabel('Y velocity (m/s)');
        title('Measured Velocities (Linear Velocities)');


        % Plot measured velocities against commanded velocities
        figure
        subplot(1,2,1)
        scatter(cmd_lin_vel_list_global, Vision_lin_vel_list_global, 'Marker', '.');
        xlabel('Commanded Linear Velocity (m/s)');
        ylabel('Measured Linear Velocity (m/s)');
        hold on
        x = [0, 2, 4];
        y = x;
        plot(x, y, 'r')
        axis equal

        subplot(1,2,2)
        scatter(cmd_ang_vel_list_global, Vision_ang_vel_list_global, 'Marker', '.');
        xlabel('Commanded Angular Velocity (rad/s)');
        ylabel('Measured Angular Velocity (rad/s)');
        hold on
        x = [-6, 0, 6];
        y = x;
        plot(x, y, 'r')
        axis equal

        % Plot measured velocities against commanded velocities (For LOLCAL VX AND VY)
        figure
        subplot(1,2,1)
        scatter(cmd_lin_vel_x_list_global, Vision_lin_vel_x_local_list_global, 'Marker', '.');
        xlabel('Commanded Vx Velocity (m/s)');
        ylabel('Measured Vx Velocity (m/s)');
        hold on
        x = [-4, 1, 4];
        y = x;
        plot(x, y, 'r')
        axis equal

        subplot(1,2,2)
        scatter(cmd_lin_vel_y_list_global, Vision_lin_vel_y_local_list_global, 'Marker', '.');
        xlabel('Commanded Vy Velocity (m/s)');
        ylabel('Measured Vy Velocity (m/s)');
        hold on
        x = [-4, 1, 4];
        y = x;
        plot(x, y, 'r')
        axis equal
    end
end








