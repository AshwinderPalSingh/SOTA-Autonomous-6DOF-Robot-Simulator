function SOTA_Factory_Simulator()

%% 1. Environment Setup
clc;
clear;
close all;

% Add the 'functions' folder to the MATLAB path
addpath('functions');

% Load the robot's parameters (UR5)
robot = define_ur5_robot();
robot.poses.home = [0, -90, 0, -90, 0, 0];
robot.poses.safe_reroute = [-90, -90, -90, -90, 90, 0]; % "Straight up"

% --- Define World & Factory ---
world_table_z = 0; 
world_hover_height = 100; % 100mm above objects
world_pick_rpy_deg = [180, 0, 0]; % Pointing down

% Pick & Place Zones
world_pick_zone = [-100, -350, 200, -150]; % [xmin, ymin, xmax, ymax]
world_place_zones = {
    [100, 150, 200, 250],   % Place Zone A
    [100, 250, 200, 350],   % Place Zone B
    [250, 150, 350, 250],   % Place Zone C
    [250, 250, 350, 350]    % Place Zone D
};
world_part_colors = {[0.8, 0, 0.2], [0, 0.8, 0.2], [0.2, 0, 0.8], [0.8, 0.8, 0]};
world_part_size = [40, 40, 40];

% --- SOTA: Collision Obstacle ---
world_obstacle.center = [0, 0]; % X, Y
world_obstacle.radius = 150;     % mm
world_obstacle.height = 400;     % mm

% --- Define App State ---
state_q_deg = robot.poses.home; % Current angles
state_object_is_gripped = false;
state_gripped_object = []; % Will store the gripped task struct
state_saved_path = {}; 
state_e_stop_triggered = false;
state_gripper_handles = [];
is_initialized = false; % *** V12.1 FIX ***

% --- SOTA: Task Queue & Vision State ---
state_task_queue = {}; % The "ground truth" of all parts
state_detected_objects = {}; % What the "vision system" has found

% --- SOTA: Analytics State ---
analytics_total_energy = 0;
analytics_start_time = 0;

% --- V8.0: Animation Engine State ---
anim_queue = {}; % MASTER QUEUE (CELL ARRAY OF STRUCTS)
anim_timer = []; % The timer object
anim_is_sensing = false; % For visual sensor cone
anim_gripper_current = 0.0; % 0=open, 1=closed
anim_gripper_target = 0.0;
anim_gripper_step = 0.1; % 10 frames to open/close


%% 2. Create the GUI Figure
UIFigure = uifigure('Name', 'SOTA Autonomous Factory Simulator (V12.1)', ...
                    'Position', [100, 100, 1400, 800], ...
                    'Visible', 'on', ... % Make visible
                    'CloseRequestFcn', @(src, event) close_figure_fcn());

% --- 3D Plot Area ---
UIAxes = uiaxes(UIFigure, 'Position', [20, 20, 860, 760]);

% --- Control Panel (Main) ---
ControlPanel = uipanel(UIFigure, 'Title', 'Control Center', 'Position', [900, 20, 480, 760]);

% --- Tab Group ---
TabGroup = uitabgroup(ControlPanel, 'Position', [10, 460, 460, 290]);

% --- TAB 1: Factory Control (NEW) ---
FactoryTab = uitab(TabGroup, 'Title', 'Factory Control');

FactoryPanel = uipanel(FactoryTab, 'Title', 'Autonomous Cycle Control', 'Position', [10, 10, 430, 240]);
SpawnPartsButton = uibutton(FactoryPanel, 'Text', '1. Spawn New Parts', 'FontWeight', 'bold', 'Position', [20, 180, 190, 40], ...
    'ButtonPushedFcn', @(src, event) spawn_parts());
NumPartsSlider = uislider(FactoryPanel, 'Limits', [1, 5], 'Value', 3, 'MajorTicks', [1, 2, 3, 4, 5], 'MinorTicks', [], 'Position', [230, 195, 180, 3]);
NumPartsLabel = uilabel(FactoryPanel, 'Text', 'Parts to Spawn: 3', 'Position', [230, 175, 180, 22]);
NumPartsSlider.ValueChangedFcn = @(src, event) update_num_parts_label(src.Value);

ScanButton = uibutton(FactoryPanel, 'Text', '2. Run Vision Scan', 'Position', [20, 130, 190, 40], ...
    'ButtonPushedFcn', @(src, event) run_vision_scan());
OptimizeCheckbox = uicheckbox(FactoryTab, 'Text', 'Optimize Task Order (TSP)', 'Value', true, 'Position', [240, 140, 180, 22]);

RunCycleButton = uibutton(FactoryPanel, 'Text', '3. Run Autonomous Cycle', 'Position', [20, 80, 390, 40], ...
    'FontWeight', 'bold', 'BackgroundColor', [0, 0.6, 0.2], 'FontColor', 'white', ...
    'ButtonPushedFcn', @(src, event) run_autonomous_cycle());
EStopButton = uibutton(FactoryPanel, 'Text', 'EMERGENCY STOP', 'Position', [20, 20, 280, 40], ...
    'FontWeight', 'bold', 'BackgroundColor', [0.8, 0, 0], 'FontColor', 'white', ...
    'ButtonPushedFcn', @(src, event) trigger_e_stop());
ResetEStopButton = uibutton(FactoryPanel, 'Text', 'Reset', 'Position', [310, 20, 100, 40], ...
    'ButtonPushedFcn', @(src, event) reset_e_stop(), 'Enable', 'off');

% --- TAB 2: Manual Control ---
ManualTab = uitab(TabGroup, 'Title', 'Manual Control');
Sliders = gobjects(robot.dof, 1);
AngleLabels = gobjects(robot.dof, 1);
slider_y_pos = 180; 
for i = 1:robot.dof
    uilabel(ManualTab, 'Text', ['Q', num2str(i), ':'], 'Position', [20, slider_y_pos, 40, 22]);
    AngleLabels(i) = uilabel(ManualTab, 'Text', [num2str(robot.poses.home(i)), ' deg'], 'Position', [400, slider_y_pos, 50, 22]);
    Sliders(i) = uislider(ManualTab, ...
        'Limits', [-180, 180], 'Value', robot.poses.home(i), ...
        'Position', [60, slider_y_pos + 10, 330, 3], ...
        'MajorTicks', [-180, -90, 0, 90, 180], ...
        'ValueChangedFcn', @(src, event) manual_slider_update(i));
    slider_y_pos = slider_y_pos - 30;
end
ResetButton = uibutton(ManualTab, 'Text', 'Reset Robot & Factory', 'Position', [100, 10, 250, 30], ...
    'ButtonPushedFcn', @(src, event) reset_to_home());

% --- TAB 3: Cartesian (IK) Control ---
IKTab = uitab(TabGroup, 'Title', 'Cartesian (IK) Control');
uilabel(IKTab, 'Text', 'Enter Target End-Effector Pose:', 'FontWeight', 'bold', 'Position', [20, 210, 400, 22]);
uilabel(IKTab, 'Text', 'X (mm):', 'Position', [20, 180, 100, 22]);
IK_X = uieditfield(IKTab, 'numeric', 'Value', 0, 'Position', [130, 180, 300, 22]);
uilabel(IKTab, 'Text', 'Y (mm):', 'Position', [20, 150, 100, 22]);
IK_Y = uieditfield(IKTab, 'numeric', 'Value', 0, 'Position', [130, 150, 300, 22]);
uilabel(IKTab, 'Text', 'Z (mm):', 'Position', [20, 120, 100, 22]);
IK_Z = uieditfield(IKTab, 'numeric', 'Value', 0, 'Position', [130, 120, 300, 22]);
uilabel(IKTab, 'Text', 'Roll (deg):', 'Position', [20, 90, 100, 22]);
IK_R = uieditfield(IKTab, 'numeric', 'Value', 0, 'Position', [130, 90, 300, 22]);
uilabel(IKTab, 'Text', 'Pitch (deg):', 'Position', [20, 60, 100, 22]);
IK_P = uieditfield(IKTab, 'numeric', 'Value', 0, 'Position', [130, 60, 300, 22]);
uilabel(IKTab, 'Text', 'Yaw (deg):', 'Position', [20, 30, 100, 22]);
IK_Y = uieditfield(IKTab, 'numeric', 'Value', 0, 'Position', [130, 30, 300, 22]);
GetPoseButton = uibutton(IKTab, 'Text', 'Get Current Pose', 'Position', [20, -10, 190, 30], ...
    'ButtonPushedFcn', @(src, event) get_current_pose());
SolveIKButton = uibutton(IKTab, 'Text', 'Solve IK & Move', 'FontWeight', 'bold', 'Position', [240, -10, 190, 30], ...
    'ButtonPushedFcn', @(src, event) solve_ik_and_move_from_ui());

% --- TAB 4: Manual Programming ---
TaskTab = uitab(TabGroup, 'Title', 'Manual Programming');
ManualCmdPanel = uipanel(TaskTab, 'Title', 'Gripper Control', 'Position', [10, 120, 430, 120]);
OpenGripperButton = uibutton(ManualCmdPanel, 'Text', 'Open Gripper', 'Position', [50, 40, 150, 40], ...
    'ButtonPushedFcn', @(src, event) open_gripper());
CloseGripperButton = uibutton(ManualCmdPanel, 'Text', 'Close Gripper', 'Position', [230, 40, 150, 40], ...
    'ButtonPushedFcn', @(src, event) close_gripper());
TeachPanel = uipanel(TaskTab, 'Title', 'Path Teaching', 'Position', [10, -80, 430, 190]);
SavePoseButton = uibutton(TeachPanel, 'Text', 'Save Current Pose', 'FontWeight', 'bold', 'Position', [20, 120, 390, 40], ...
    'ButtonPushedFcn', @(src, event) save_current_pose());
RunPathButton = uibutton(TeachPanel, 'Text', 'Run Saved Path', 'Position', [20, 70, 190, 30], ...
    'ButtonPushedFcn', @(src, event) run_saved_path());
ClearPathButton = uibutton(TeachPanel, 'Text', 'Clear Saved Path', 'Position', [220, 70, 190, 30], ...
    'ButtonPushedFcn', @(src, event) clear_saved_path());
PathStatusLabel = uilabel(TeachPanel, 'Text', 'Saved Poses: 0', 'Position', [20, 30, 390, 22]);

% --- SOTA: Analytics Panel (New) ---
AnalyticsPanel = uipanel(ControlPanel, 'Title', 'SOTA: Performance Analytics', 'Position', [10, 390, 460, 60]);
AnalyticsCycleTime = uilabel(AnalyticsPanel, 'Text', 'Cycle Time (sec):  --', 'Position', [20, 10, 200, 22]);
AnalyticsEnergy = uilabel(AnalyticsPanel, 'Text', 'Energy Used (kWh): --', 'Position', [240, 10, 200, 22]);

% --- SOTA: Fault/Safety Panel (Modified) ---
FaultPanel = uipanel(ControlPanel, 'Title', 'SOTA: Fault Injection & Safety', 'Position', [10, 320, 460, 60]);
FailureSlider = uislider(FaultPanel, 'Limits', [0, 100], 'Value', 0, 'Position', [20, 20, 280, 3]);
FailureLabel = uilabel(FaultPanel, 'Text', 'Grip Failure Chance: 0%', 'Position', [310, 10, 140, 22]);
FailureSlider.ValueChangedFcn = @(src, event) update_failure_label(src.Value);

% --- View Control Panel (Retained) ---
ViewPanel = uipanel(ControlPanel, 'Title', 'View Controls', 'Position', [10, 260, 460, 50]);
IsoButton = uibutton(ViewPanel, 'Text', 'ISO View', 'Position', [20, 10, 130, 30], ...
    'ButtonPushedFcn', @(src, event) set_view('iso'));
TopButton = uibutton(ViewPanel, 'Text', 'Top View (X-Y)', 'Position', [160, 10, 130, 30], ...
    'ButtonPushedFcn', @(src, event) set_view('top'));
SideButton = uibutton(ViewPanel, 'Text', 'Side View (Y-Z)', 'Position', [300, 10, 130, 30], ...
    'ButtonPushedFcn', @(src, event) set_view('side'));
CollisionCheck = uicheckbox(ControlPanel, 'Text', 'Collisions On', 'Position', [310, 275, 120, 22], 'Value', true); % Moved


% --- Command Log Panel (Resized) ---
LogPanel = uipanel(ControlPanel, 'Title', 'Factory Log', 'Position', [10, 70, 460, 180]);
LogArea = uitextarea(LogPanel, 'Value', {'Initializing...'}, 'Editable', 'off', 'Position', [5, 5, 450, 150]);

% --- Status Panel (Readout) ---
StatusPanel = uipanel(ControlPanel, 'Title', 'End-Effector Pose', 'Position', [10, 10, 460, 50]);
uilabel(StatusPanel, 'Text', 'X:', 'Position', [10, 10, 20, 22]);
XPos = uieditfield(StatusPanel, 'numeric', 'Value', 0, 'Position', [35, 10, 80, 22], 'Editable', 'off');
uilabel(StatusPanel, 'Text', 'Y:', 'Position', [125, 10, 20, 22]);
YPos = uieditfield(StatusPanel, 'numeric', 'Value', 0, 'Position', [150, 10, 80, 22], 'Editable', 'off');
uilabel(StatusPanel, 'Text', 'Z:', 'Position', [240, 10, 20, 22]);
ZPos = uieditfield(StatusPanel, 'numeric', 'Value', 0, 'Position', [265, 10, 80, 22], 'Editable', 'off');
PoseStatus = uilabel(StatusPanel, 'Text', '(R,P,Y will go here)', 'Position', [350, 10, 150, 22]);

%% 3. GUI STARTUP FUNCTION (V11.1 FIX)
% The timer is created here, *after* the UI is built,
% and given a StartDelay to ensure all components are valid
% before the first call to animation_step_fcn.

% --- V11.1: Animation Timer ---
anim_timer = timer(...
    'ExecutionMode', 'fixedRate', ...
    'Period', 0.033, ... % 30 FPS
    'StartDelay', 0.5, ... % *** V11.1 FIX: Wait 0.5s before first run
    'TimerFcn', @(src, event) animation_step_fcn());
% Add a cleanup function to stop the timer when the figure is closed
UIFigure.CloseRequestFcn = @(src, event) close_figure_fcn();

% Start the timer. It will wait 0.5s, then run animation_step_fcn
% for the first time, which will handle initialization.
start(anim_timer);


%% 4. CALLBACK FUNCTIONS (Nested)

% --- V8.0: Figure Close Function ---
function close_figure_fcn()
    try
        stop(anim_timer); % Stop the timer
        delete(anim_timer); % Delete the timer
    catch
        % Timer may already be deleted, ignore error
    end
    delete(UIFigure); % Close the figure
end

% --- Log Helper ---
function log_message(msg)
    % Check if LogArea is valid (it might not be during startup)
    if ~isempty(LogArea) && isvalid(LogArea)
        LogArea.Value = [{['[' datestr(now, 'HH:MM:SS') '] ' msg]}; LogArea.Value];
        drawnow; % Force log to update
    else
        disp(msg); % Fallback to command window
    end
end

% --- Slider Label Callbacks ---
function update_num_parts_label(value)
    NumPartsLabel.Text = ['Parts to Spawn: ' num2str(round(value))];
end

function update_failure_label(value)
    FailureLabel.Text = ['Grip Failure: ' num2str(round(value)) '%'];
end

% --- View Callbacks ---
function set_view(view_type)
    if strcmp(view_type, 'iso')
        view(UIAxes, 120, 20);
        log_message('View set to Isometric.');
    elseif strcmp(view_type, 'top')
        view(UIAxes, 0, 90); % X-Y
        log_message('View set to Top (X-Y).');
    elseif strcmp(view_type, 'side')
        view(UIAxes, 90, 0); % Y-Z
        log_message('View set to Side (Y-Z).');
    end
end

% --- SOTA: Factory Control Callbacks ---
function spawn_parts()
    log_message('Spawning new parts...');
    state_task_queue = {}; % Clear old tasks
    state_detected_objects = {}; % Clear vision
    FactoryStatus.Items = {'New parts spawned. Ready for vision scan.'};
    
    num_parts = round(NumPartsSlider.Value);
    
    for i = 1:num_parts
        % Random pick position
        zone = world_pick_zone;
        pick_x = zone(1) + (zone(3) - zone(1)) * rand();
        pick_y = zone(2) + (zone(4) - zone(2)) * rand();
        pick_pos = [pick_x, pick_y, world_table_z];
        
        % Random place zone
        zone_idx = mod(i-1, length(world_place_zones)) + 1;
        zone = world_place_zones{zone_idx};
        place_x = zone(1) + (zone(3) - zone(1)) * rand();
        place_y = zone(2) + (zone(4) - zone(2)) * rand();
        place_pos = [place_x, place_y, world_table_z];
        
        % Create task
        task.id = sprintf('Part_%d', i);
        task.pick_pos = pick_pos; % This is the "home" position
        task.place_pos = place_pos;
        task.color = world_part_colors{zone_idx};
        task.size = world_part_size;
        
        state_task_queue{end+1} = task;
    end
    
    log_message(sprintf('Spawned %d new parts in pick zone.', num_parts));
    update_simulation_view(state_q_deg);
end

function run_vision_scan()
    if isempty(state_task_queue)
        log_message('Vision Error: No parts to scan. Please spawn parts first.');
        return;
    end
    
    log_message('Vision System: Scanning for objects...');
    state_detected_objects = {}; % Clear old detections
    
    % Simulate scanning
    pause(0.5);
    
    for i = 1:length(state_task_queue)
        task = state_task_queue{i};
        % Simulate adding noise to detection
        detected_pos = task.pick_pos + (rand(1,3)-0.5) * 5; % +/- 2.5mm noise
        
        detection.id = task.id;
        detection.pos = detected_pos;
        detection.size = task.size;
        detection.color = task.color;
        detection.place_pos = task.place_pos; % Pass this along
        
        state_detected_objects{end+1} = detection;
        log_message(sprintf('Vision System: Found %s at [%.1f, %.1f, %.1f]', ...
            detection.id, detection.pos(1), detection.pos(2), detection.pos(3)));
    end
    
    FactoryStatus.Items = {sprintf('Scan complete. Found %d objects.', length(state_detected_objects)); 'Ready for autonomous cycle.'};
    update_simulation_view(state_q_deg); % Redraw to show detection boxes
end

% --- SOTA: Autonomous Cycle (with Optimization) ---
function run_autonomous_cycle()
    if isempty(state_detected_objects)
        log_message('Cycle Error: No objects detected. Please run vision scan.');
        return;
    end
    
    log_message(sprintf('--- AUTONOMOUS CYCLE BAŞLADI: %d tasks to process ---', length(state_detected_objects)));
    set_controls_enabled('off');
    state_e_stop_triggered = false;
    
    % --- SOTA: Analytics ---
    analytics_total_energy = 0;
    analytics_start_time = tic;
    
    % --- SOTA: Task Optimization ---
    tasks_to_process = state_detected_objects;
    if OptimizeCheckbox.Value
        log_message('Optimizing task order (TSP Nearest Neighbor)...');
        tasks_to_process = optimize_task_order_tsp(tasks_to_process);
        % Log new order
        order_str = 'Optimized Order: ';
        for i = 1:length(tasks_to_process)
            order_str = [order_str, tasks_to_process{i}.id, '->'];
        end
        log_message(order_str(1:end-2));
    else
        log_message('Processing tasks in scanned order.');
    end
    
    state_detected_objects = {}; % Clear detections list
    num_tasks = length(tasks_to_process);

    % --- V8.0: Build Master Animation Queue ---
    master_queue = {};
    q_current = state_q_deg;
    
    for i = 1:num_tasks
        task = tasks_to_process{i};
        FactoryStatus.Items = {sprintf('Planning Task %d/%d: %s', i, num_tasks, task.id)};
        
        % Get the full plan for this one task
        [task_queue_segment, q_end, success] = plan_single_pnp_task(task, q_current);
        
        if ~success
            log_message(sprintf('Failed to plan task %s. Skipping.', task.id));
            
            % *** V11.1 BUGFIX: Update task queue even on failure ***
            % Find and update the original task in the task_queue
            for j = 1:length(state_task_queue)
                if strcmp(state_task_queue{j}.id, task.id)
                    % Don't update position, just log skip
                    log_message(sprintf('Task %s failed and was skipped.', task.id));
                    break;
                end
            end
            continue; % Skip to next task
        end
        
        master_queue = [master_queue; task_queue_segment];
        q_current = q_end; % Update start for next loop
        
        % Add a return-to-home move
        if i < num_tasks
            [traj_home, q_end_home] = plan_move(q_current, robot.poses.home, 40);
            if isempty(traj_home)
                log_message('Failed to plan return to home. Aborting cycle.');
                break;
            end
            master_queue = [master_queue; traj_home];
            q_current = q_end_home;
        end
    end
    
    % --- V8.0: Run the Master Animation ---
    log_message('Planning complete. Executing master animation queue...');
    run_animation_queue(master_queue);
end

% --- SOTA: Task Optimization Function (TSP) ---
function sorted_list = optimize_task_order_tsp(detected_list)
    % Greedy Nearest Neighbor algorithm for TSP
    num_tasks = length(detected_list);
    sorted_list = cell(1, num_tasks);
    remaining_tasks = detected_list;
    
    % Get home XY position
    [T_home, ~] = compute_fk(robot, robot.poses.home);
    current_pos_xy = T_home(1:2, 4)'; % 1x2 [x, y]
    
    for i = 1:num_tasks
        min_dist = inf;
        best_idx = -1;
        
        % Find the closest task to the *current* position
        for j = 1:length(remaining_tasks)
            task_pos_xy = remaining_tasks{j}.pos(1:2);
            dist = norm(current_pos_xy - task_pos_xy);
            if dist < min_dist
                min_dist = dist;
                best_idx = j;
            end
        end
        
        % Add the closest task to the sorted list
        closest_task = remaining_tasks{best_idx};
        sorted_list{i} = closest_task;
        
        % Update current position to this task's *place* position
        % (This simulates the full travel path)
        current_pos_xy = closest_task.place_pos(1:2);
        
        % Remove it from the remaining list
        remaining_tasks(best_idx) = [];
    end
end

% --- SOTA: E-Stop Functions ---
function trigger_e_stop()
    state_e_stop_triggered = true;
    stop(anim_timer); % V8.0: Immediately stop the animation
    anim_queue = {}; % Clear the animation queue
    set_controls_enabled('e_stop'); % Call the new e-stop state
    log_message('!!! EMERGENCY STOP TRIGGERED !!!');
end

function reset_e_stop()
    state_e_stop_triggered = false;
    set_controls_enabled('on'); % Re-enable all controls
    start(anim_timer); % V10.2: Restart the timer
    log_message('E-Stop Reset. System ready.');
end

% --- V8.0: Single Task *Planner* (no animation) ---
function [task_queue, q_final, success] = plan_single_pnp_task(task, q_start)
    task_queue = {};
    q_final = q_start;
    success = false; % Assume failure
    
    % 1. Define Poses from 'task'
    rpy_target = world_pick_rpy_deg;
    pos_pick_hover = task.pos + [0, 0, world_hover_height];
    pos_pick_grasp = task.pos + [0, 0, task.size(3)]; % Target top of cube
    pos_place_hover = task.place_pos + [0, 0, world_hover_height];
    pos_place_release = task.place_pos + [0, 0, task.size(3)];
    
    % 2. Solve for all required joint angles
    q_pick_hover = solve_ik_for_task(pos_pick_hover, rpy_target, q_start);
    if isempty(q_pick_hover), log_message('Aborted: Pick Hover Unreachable.'); return; end
    
    q_pick_grasp = solve_ik_for_task(pos_pick_grasp, rpy_target, q_pick_hover);
    if isempty(q_pick_grasp), log_message('Aborted: Pick Grasp Unreachable.'); return; end
    
    q_place_hover = solve_ik_for_task(pos_place_hover, rpy_target, q_pick_grasp);
    if isempty(q_place_hover), log_message('Aborted: Place Hover Unreachable.'); return; end
    
    q_place_release = solve_ik_for_task(pos_place_release, rpy_target, q_place_hover);
    if isempty(q_place_release), log_message('Aborted: Place Release Unreachable.'); return; end
    
    % 3. Build the Master Queue for this *one* task
    [traj1, q_end1] = plan_move(q_start, q_pick_hover, 50);
    [traj_sense_down, q_end2] = plan_sensing_move(q_end1, pos_pick_grasp(3), 'down');
    
    % SOTA: Add the (potential) grip failure command
    grip_frame.q_deg = q_end2;
    grip_frame.command = 'GripClose';
    grip_frame.payload = task; % Send the whole task
    grip_frame.retries = 0; % First attempt
    
    [traj_sense_up, q_end4] = plan_sensing_move(q_end2, pos_pick_hover(3), 'up');
    [traj_move_over, q_end5] = plan_move(q_end4, q_place_hover, 50);
    [traj_sense_release, q_end6] = plan_sensing_move(q_end5, pos_place_release(3), 'down');
    
    % Frame to open gripper
    release_frame.q_deg = q_end6;
    release_frame.command = 'GripOpen';
    release_frame.payload = task; % V11.1 FIX: Pass task to update queue
    release_frame.retries = 0;
    
    [traj_sense_up_final, q_end7] = plan_sensing_move(q_end6, pos_place_hover(3), 'up');
    
    % Build the final queue
    task_queue = [
        traj1; 
        traj_sense_down;
        {grip_frame}; % Add the single grip command
        plan_gripper_wait(q_end2); % Wait 10 frames for grip
        traj_sense_up;
        traj_move_over;
        traj_sense_release;
        {release_frame}; % Add the single release command
        plan_gripper_wait(q_end6); % Wait 10 frames for release
        traj_sense_up_final
    ];
    
    q_final = q_end7; % The final pose after this task
    success = true; % Planning was successful
end

% --- SOTA: Collision-Aware Path Planner (Refined) ---
function [queue, q_end] = plan_move(q_start, q_end, steps)
    
    direct_traj = generate_trajectory(q_start, q_end, steps);
    
    % Collision Check
    is_safe = true;
    if CollisionCheck.Value
        is_safe = check_trajectory_collision(direct_traj);
    end
    
    final_traj = [];
    if is_safe
        final_traj = direct_traj;
    else
        log_message('Collision Detected! Rerouting via safe point...');
        q_safe_midpoint = robot.poses.safe_reroute; % "Straight up" pose
        
        % Check path to midpoint
        traj1 = generate_trajectory(q_start, q_safe_midpoint, 30);
        if CollisionCheck.Value && ~check_trajectory_collision(traj1)
            log_message('Reroute Error: Path to safe point is also blocked!');
            queue = {}; q_end = q_start;
            return;
        end
        
        % Check path from midpoint
        traj2 = generate_trajectory(q_safe_midpoint, q_end, 30);
        if CollisionCheck.Value && ~check_trajectory_collision(traj2)
            log_message('Reroute Error: Path from safe point is blocked!');
            queue = {}; q_end = q_start;
            return;
        end
        
        final_traj = [traj1; traj2];
    end
    
    % Convert numeric trajectory to cell array of structs
    num_frames = size(final_traj, 1);
    queue = cell(num_frames, 1);
    for k = 1:num_frames
        frame.q_deg = final_traj(k, :);
        frame.command = 'None';
        frame.payload = [];
        frame.retries = 0;
        queue{k} = frame;
    end
    
    q_end = final_traj(end, :);
    
    % SOTA: Analytics - Calculate energy for this move
    q_diff = abs(diff(final_traj, 1, 1)); % Difference between steps
    joint_travel_deg = sum(q_diff, 'all'); % Total degrees moved by all joints
    energy_kwh = joint_travel_deg * 0.00001; % Arbitrary energy factor
    analytics_total_energy = analytics_total_energy + energy_kwh;
end

% --- SOTA: Collision Checking Function (with Table check) ---
function is_safe = check_trajectory_collision(trajectory)
    is_safe = true;
    obstacle = world_obstacle;
    
    % Check every 5th step for speed
    for k = 1:5:size(trajectory, 1)
        q_step = trajectory(k, :);
        [~, joint_positions] = compute_fk(robot, q_step);
        
        % Check all links (points 3, 4, 5, 6)
        for j = 4:size(joint_positions, 2) % Check upper arm
            pos = joint_positions(:, j);
            
            % Check for table collision
            if pos(3) < world_table_z - 1 % 1mm tolerance
                is_safe = false;
                log_message(sprintf('Safety Error: Table collision detected at Joint %d!', j-1));
                return;
            end

            % Check if inside the obstacle cylinder
            if pos(3) < obstacle.height % Below top of obstacle
                dist_from_center = norm(pos(1:2) - obstacle.center');
                if dist_from_center < obstacle.radius
                    is_safe = false;
                    log_message(sprintf('Safety Error: Obstacle collision detected at Joint %d!', j-1));
                    return; % A collision was found
                end
            end
        end
    end
end

% --- SOTA: Sensor-Based Move Planner ---
function [traj_queue, q_final] = plan_sensing_move(q_start, target_z, direction)
    
    % Get current pose
    [T_current, ~] = compute_fk(robot, q_start);
    current_pos = T_current(1:3, 4)';
    current_rpy = world_pick_rpy_deg;
    
    % Define start and end points for this move
    pos_start = current_pos;
    pos_end = [current_pos(1), current_pos(2), target_z];
    
    % Create a short, linear trajectory for the Z-move
    num_steps = 15;
    z_traj = linspace(pos_start(3), pos_end(3), num_steps)';
    
    % Solve IK for each step of this linear move
    q_traj = zeros(num_steps, 6);
    q_last_good = q_start;
    
    for k = 1:num_steps
        pos_step = [pos_start(1), pos_start(2), z_traj(k)];
        q_sol = solve_ik_for_task(pos_step, current_rpy, q_last_good);
        if isempty(q_sol)
            log_message(sprintf('Sensing Move Error: IK failed at Z=%.1f', z_traj(k)));
            traj_queue = {}; q_final = q_start;
            return;
        end
        q_traj(k, :) = q_sol;
        q_last_good = q_sol;
    end
    
    % V8.0: Convert to cell array of structs
    traj_queue = cell(num_steps + 10, 1); % 10 hold frames
    sense_cmd = 'SenseDown';
    if strcmp(direction, 'up'), sense_cmd = 'SenseUp'; end
    
    % 5 "Sensing" frames
    for k = 1:5
        frame.q_deg = q_traj(1, :);
        frame.command = sense_cmd;
        frame.payload = [];
        frame.retries = 0;
        traj_queue{k} = frame;
    end
    
    % 15 "Move" frames
    for k = 1:num_steps
        frame.q_deg = q_traj(k, :);
        frame.command = 'None';
        frame.payload = [];
        frame.retries = 0;
        traj_queue{k+5} = frame;
    end
    
    % 5 "Sensing" frames at end
    for k = 1:5
        frame.q_deg = q_traj(end, :);
        frame.command = sense_cmd;
        frame.payload = [];
        frame.retries = 0;
        traj_queue{k+5+num_steps} = frame;
    end
    
    q_final = q_traj(end, :);
end

% --- V8.0: Gripper Wait Planner ---
function wait_queue = plan_gripper_wait(q_pose)
    % This function creates a "fake" queue of 10 frames
    % that just hold the robot's position.
    wait_queue = cell(10, 1);
    for k = 1:10
        frame.q_deg = q_pose;
        frame.command = 'None';
        frame.payload = [];
        frame.retries = 0;
        wait_queue{k} = frame;
    end
end


% --- Manual Tab Callbacks ---
function manual_slider_update(slider_index)
    q_val = Sliders(slider_index).Value;
    q_new = state_q_deg;
    q_new(slider_index) = q_val;
    
    % V8.0: Use the animation queue for manual moves
    [traj, ~] = plan_move(state_q_deg, q_new, 2); % 2-step move
    run_animation_queue(traj);
end

% --- IK Tab Callbacks ---
function get_current_pose()
    log_message('Getting current end-effector pose...');
    [T_final, ~] = compute_fk(robot, state_q_deg);
    pos = T_final(1:3, 4);
    [r, p, y] = tform_to_rpy(T_final(1:3, 1:3));
    
    IK_X.Value = round(pos(1), 2);
    IK_Y.Value = round(pos(2), 2);
    IK_Z.Value = round(pos(3), 2);
    IK_R.Value = round(r, 2);
    IK_P.Value = round(p, 2);
    IK_Y.Value = round(y, 2);
    log_message('Current pose populated in IK fields.');
end

function solve_ik_and_move_from_ui()
    log_message('Solving IK from UI...');
    x = IK_X.Value; y = IK_Y.Value; z = IK_Z.Value;
    r = IK_R.Value; p = IK_P.Value; yw = IK_Y.Value;
    
    q_target = solve_ik_for_task([x, y, z], [r, p, yw], state_q_deg);
    
    if isempty(q_target)
        log_message('IK move failed.');
    else
        log_message('IK move complete. Planning path...');
        [traj, ~] = plan_move(state_q_deg, q_target, 50);
        run_animation_queue(traj);
    end
end

% --- Manual Programming Tab Callbacks ---
function reset_to_home()
    log_message('Resetting factory and moving to Home...');
    set_controls_enabled('off');
    
    % Reset factory state
    state_task_queue = {};
    state_detected_objects = {};
    state_object_is_gripped = false;
    state_gripped_object = [];
    FactoryStatus.Items = {'Factory Reset. Idle.'};
    
    [traj, ~] = plan_move(state_q_deg, robot.poses.home, 40);
    run_animation_queue(traj);
end

function open_gripper()
    log_message('Gripper Opening...');
    anim_gripper_target = 0.0; % 0 = open
    state_object_is_gripped = false;
    state_gripped_object = [];
end

function close_gripper()
    log_message('Gripper Closing (Manual)...');
    anim_gripper_target = 1.0; % 1 = closed
    
    % Manual grip check
    [T_final, ~] = compute_fk(robot, state_q_deg);
    pos = T_final(1:3, 4);
    
    % Check proximity to all *spawned* tasks
    task_to_remove_idx = -1;
    for i = 1:length(state_task_queue)
        task = state_task_queue{i};
        if isempty(task), continue; end
        
        task_top_pos = task.pick_pos + [0, 0, task.size(3)];
        if norm(pos' - task_top_pos) < 20 % 20mm tolerance
            state_object_is_gripped = true;
            state_gripped_object = task; % Store for color
            task_to_remove_idx = i;
            log_message(sprintf('Manually gripped %s.', task.id));
            break;
        end
    end
    
    % Remove the task from the queue *after* the loop
    if task_to_remove_idx > 0
        state_task_queue(task_to_remove_idx) = [];
    end
end

function save_current_pose()
    current_q = state_q_deg;
    state_saved_path{end+1} = current_q;
    num_poses = length(state_saved_path);
    PathStatusLabel.Text = ['Saved Poses: ' num2str(num_poses)];
    log_message(sprintf('Pose %d saved.', num_poses));
end

function run_saved_path()
    num_poses = length(state_saved_path);
    if num_poses < 2
        log_message('Error: Need at least 2 poses to run a path.');
        return;
    end
    log_message(sprintf('--- Running saved path with %d poses ---', num_poses));
    set_controls_enabled('off');
    
    q_current = state_q_deg;
    master_queue = {};
    
    for i = 1:num_poses
        log_message(sprintf('Planning move to Pose %d...', i));
        [traj, q_end] = plan_move(q_current, state_saved_path{i}, 40);
        if isempty(traj)
            log_message('Failed to plan path. Aborting.');
            set_controls_enabled('on');
            return;
        end
        master_queue = [master_queue; traj];
        q_current = q_end;
    end
    
    run_animation_queue(master_queue);
end

function clear_saved_path()
    state_saved_path = {};
    PathStatusLabel.Text = 'Saved Poses: 0';
    log_message('Saved path cleared.');
end


%% 5. CORE FUNCTIONS (Nested)

% --- Helper for IK ---
function q_best = solve_ik_for_task(target_pos, target_rpy_deg, q_previous)
    T_target = rpy_to_tform(target_pos, deg2rad(target_rpy_deg));
    q_solutions_rad = solve_ur5_ik(T_target, robot);
    
    if isempty(q_solutions_rad)
        q_best = [];
        return;
    end
    
    q_solutions_deg = rad2deg(q_solutions_rad);
    q_best = q_solutions_deg(1, :);
    min_dist = norm(q_previous - q_best);
    
    for k = 2:size(q_solutions_deg, 1)
        dist = norm(q_previous - q_solutions_deg(k, :));
        if dist < min_dist
            min_dist = dist;
            q_best = q_solutions_deg(k, :);
        end
    end
end

% --- V8.0: Animation Queue Runner ---
function run_animation_queue(queue)
    if isempty(queue)
        log_message('Animation queue is empty. Nothing to do.');
        set_controls_enabled('on'); % Re-enable if we were called
        return;
    end
    
    % Add the final pose to the queue for 1 extra frame to ensure it renders
    queue{end+1} = queue{end}; 
    
    anim_queue = [anim_queue; queue]; % Append to master queue
    
    % Start the timer if it's not already running
    if strcmp(anim_timer.Running, 'off')
        start(anim_timer);
    end
end

% --- V8.0: Main Animation Timer Callback (The "Game Loop") ---
function animation_step_fcn()
    
    % --- V11.1: One-time initialization ---
    if ~is_initialized
        log_message('Simulator ready.');
        update_simulation_view(state_q_deg);
        is_initialized = true;
        stop(anim_timer); % Stop, wait for a job
        return;
    end
    
    % Check for E-Stop
    if state_e_stop_triggered
        stop(anim_timer);
        anim_queue = {}; % Clear queue
        return;
    end
    
    % Check if queue is empty
    if isempty(anim_queue)
        % Don't stop the timer, just idle
        % stop(anim_timer); 
        
        % Only run completion logic ONCE
        if analytics_start_time ~= 0
            set_controls_enabled('on');
            log_message('Animation queue finished.');
            
            % --- SOTA: Report Analytics ---
            total_time = toc(analytics_start_time);
            AnalyticsCycleTime.Text = sprintf('Cycle Time (sec):  %.2f', total_time);
            AnalyticsEnergy.Text = sprintf('Energy Used (kWh): %.4f', analytics_total_energy);
            log_message(sprintf('--- CYCLE ANALYTICS ---'));
            log_message(sprintf('Total Time: %.2f sec', total_time));
            log_message(sprintf('Total Energy: %.4f kWh', analytics_total_energy));
            
            analytics_start_time = 0; % Reset for next run
            
            if state_e_stop_triggered
                log_message('--- CYCLE ABORTED (E-STOP) ---');
                FactoryStatus.Items = {'CYCLE ABORTED (E-STOP)'};
            else
                log_message('--- AUTONOMOUS CYCLE COMPLETE ---');
                FactoryStatus.Items = {'Autonomous Cycle Complete. Factory Idle.'};
            end
        end
        
        % Keep rendering the current state
        update_simulation_view(state_q_deg);
        return;
    end
    
    % Get the next frame from the queue
    frame_data = anim_queue{1};
    anim_queue(1) = []; % Pop from queue
    q_step = frame_data.q_deg;
    
    % Handle special command flags
    anim_is_sensing = false; % Default
    
    switch frame_data.command
        case 'SenseDown'
            anim_is_sensing = true;
        case 'SenseUp'
            anim_is_sensing = true;
        case 'GripClose'
            % --- SOTA: Runtime Fault & Recovery ---
            if rand() < (FailureSlider.Value / 100) && frame_data.retries < 3
                % GRIP FAILED
                log_message(sprintf('Grip Failed on %s! Retrying (%d/3)...', ...
                    frame_data.payload.id, frame_data.retries + 1));
                
                % Create a new retry plan
                retry_plan = {};
                [traj_up, q_up] = plan_sensing_move(q_step, world_hover_height, 'up');
                [traj_down, q_down] = plan_sensing_move(q_up, pos_from_q(q_step, [0,0,0]), 'down'); % Go back to grasp Z
                
                % Create a new grip command with incremented retry count
                new_grip_frame = frame_data;
                new_grip_frame.retries = frame_data.retries + 1;
                new_grip_frame.q_deg = q_down;
                
                retry_plan = [traj_up; traj_down; {new_grip_frame}; plan_gripper_wait(q_down)];
                
                % Pre-pend this plan to the *front* of the master queue
                anim_queue = [retry_plan; anim_queue];
                
            elseif frame_data.retries >= 3
                % CRITICAL FAILURE
                log_message(sprintf('CRITICAL FAILURE: Cannot grip %s. Skipping task.', frame_data.payload.id));
                % Don't set gripper, just let queue continue
            else
                % GRIP SUCCEEDED
                log_message(sprintf('Grip Succeeded on %s.', frame_data.payload.id));
                anim_gripper_target = 1.0;
                state_gripped_object = frame_data.payload; % Get gripped object
                state_object_is_gripped = true;
            end
            
        case 'GripOpen'
            anim_gripper_target = 0.0;
            
            % *** V11.1 FIX: Update task queue on successful drop ***
            task = frame_data.payload;
            if ~isempty(task)
                for j = 1:length(state_task_queue)
                    if strcmp(state_task_queue{j}.id, task.id)
                        state_task_queue{j}.pick_pos = task.place_pos; 
                        log_message(sprintf('Task %s complete. Object moved to new home.', task.id));
                        break;
                    end
                end
            end
            
            state_gripped_object = []; % Clear gripped object
            state_object_is_gripped = false;
    end
    
    
    % --- V7.0: Interpolate Gripper ---
    if anim_gripper_current < anim_gripper_target
        anim_gripper_current = min(anim_gripper_current + anim_gripper_step, 1.0);
    elseif anim_gripper_current > anim_gripper_target
        anim_gripper_current = max(anim_gripper_current - anim_gripper_step, 0.0);
    end
    
    % Update the simulation plot
    update_simulation_view(q_step);
    state_q_deg = q_step; % Store the last pose
end


% --- Master Plotting Function (Called by EVERY update) ---
function update_simulation_view(q_deg)
    % 1. Compute FK
    [T_final, all_positions] = compute_fk(robot, q_deg);
    
    % 2. Get End-Effector Pose
    pos = T_final(1:3, 4); 
    rot_matrix = T_final(1:3, 1:3);
    [r, p, y] = tform_to_rpy(rot_matrix);
    
    % 3. Update the Status Panels
    XPos.Value = round(pos(1), 1);
    YPos.Value = round(pos(2), 1);
    ZPos.Value = round(pos(3), 1);
    PoseStatus.Text = sprintf('R:%.1f P:%.1f Y:%.1f', r, p, y);
    
    % 4. Update Manual Sliders and Labels (if not animating)
    if strcmp(anim_timer.Running, 'off') && ~state_e_stop_triggered
        for i = 1:robot.dof
            Sliders(i).Value = q_deg(i);
            AngleLabels(i).Text = [num2str(q_deg(i), '%.1f'), ' deg'];
        end
    end
    
    % 5. Update the 3D Plot
    cla(UIAxes); 
    
    % --- Draw World Environment ---
    table_v = [ -600, -600, world_table_z; 600, -600, world_table_z; 
                 600,  600, world_table_z; -600,  600, world_table_z];
    table_f = [1, 2, 3, 4];
    patch(UIAxes, 'Vertices', table_v, 'Faces', table_f, 'FaceColor', [0.7 0.7 0.7], 'FaceAlpha', 0.5);
    hold(UIAxes, 'on');
    
    % Draw Pick/Place Zones
    plot_zone(world_pick_zone, [0.8, 0.8, 0.2]); % Yellow
    for i = 1:length(world_place_zones)
        plot_zone(world_place_zones{i}, world_part_colors{i});
    end
    
    % --- Draw SOTA: Collision Obstacle ---
    obs = world_obstacle;
    [X, Y, Z] = cylinder(obs.radius, 30);
    X = X + obs.center(1);
    Y = Y + obs.center(2);
    Z = Z * obs.height;
    surf(UIAxes, X, Y, Z, 'FaceColor', 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    
    % --- Draw SOTA: Task Objects (from queue) ---
    for i = 1:length(state_task_queue)
        task = state_task_queue{i};
        if isempty(task), continue; end
        plotcube(UIAxes, task.pick_pos, task.size, task.color);
    end
    
    % --- Draw SOTA: Detected Objects (Vision) ---
    for i = 1:length(state_detected_objects)
        detection = state_detected_objects{i};
        if isempty(detection), continue; end
        
        pos_det = detection.pos;
        sz = detection.size;
        v = [pos_det(1), pos_det(2), pos_det(3); pos_det(1)+sz(1), pos_det(2), pos_det(3); ...
             pos_det(1)+sz(1), pos_det(2)+sz(2), pos_det(3); pos_det(1), pos_det(2)+sz(2), pos_det(3); ...
             pos_det(1), pos_det(2), pos_det(3)+sz(3); pos_det(1)+sz(1), pos_det(2), pos_det(3)+sz(3); ...
             pos_det(1)+sz(1), pos_det(2)+sz(2), pos_det(3)+sz(3); pos_det(1), pos_det(2)+sz(2), pos_det(3)+sz(3)];
        f = [1,2,6,5; 2,3,7,6; 3,4,8,7; 4,1,5,8; 1,2,3,4; 5,6,7,8];
        patch(UIAxes, 'Vertices', v, 'Faces', f, ...
            'FaceColor', 'none', 'EdgeColor', [0, 1, 1], 'LineWidth', 1.5, 'LineStyle', '--');
        text(UIAxes, pos_det(1), pos_det(2), pos_det(3)+sz(3)+20, detection.id, 'Color', [0, 1, 1]);
    end

    % --- Draw Gripped Object (BUGFIXED) ---
    if state_object_is_gripped
        gripper_offset = [0; 0; -state_gripped_object.size(3)];
        obj_origin_T = T_final * [gripper_offset; 1];
        obj_pos = obj_origin_T(1:3)'; 
        color = state_gripped_object.color;
        plotcube(UIAxes, obj_pos, state_gripped_object.size, color);
    end

    % --- Draw Robot Arm ---
    x = all_positions(1, :); y = all_positions(2, :); z = all_positions(3, :);
    plot3(UIAxes, x, y, z, '-o', 'Color', [0, 0.4470, 0.7410], ...
          'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', [1, 0.5, 0]);
      
    % --- Draw Gripper (V8.0: Interpolated) ---
    gripper_len = 40;
    z_dir = rot_matrix(:, 3); y_dir = rot_matrix(:, 2);
    
    % Interpolate width: 25 (open) to 5 (closed)
    g_width = 25 - (anim_gripper_current * 20);
    
    g1_end = pos + (z_dir * gripper_len) + (y_dir * g_width);
    g2_end = pos + (z_dir * gripper_len) - (y_dir * g_width);
    
    if ~isempty(state_gripper_handles)
        delete(state_gripper_handles);
    end
    g_line1 = plot3(UIAxes, [pos(1), g1_end(1)], [pos(2), g1_end(2)], [pos(3), g1_end(3)], 'k', 'LineWidth', 4);
    g_line2 = plot3(UIAxes, [pos(1), g2_end(1)], [pos(2), g2_end(2)], [pos(3), g2_end(3)], 'k', 'LineWidth', 4);
    
    % --- V8.0: Draw Visual Sensor Cone ---
    if anim_is_sensing
        sensor_len = 60;
        sensor_radius = 15;
        [X, Y, Z] = cylinder([sensor_radius, 0], 10);
        Z = Z * sensor_len;
        % Rotate the cone
        [az, el] = view(UIAxes); % Get current view
        h = surf(UIAxes, X, Y, Z, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
        v_orig = [0; 0; 1];
        v_new = z_dir;
        axis_rot = cross(v_orig, v_new);
        angle_rot = acos(dot(v_orig, v_new));
        if norm(axis_rot) > 1e-6
             rotate(h, axis_rot, rad2deg(angle_rot), [0,0,0]);
        end
        % Translate the cone
        h.XData = h.XData + pos(1);
        h.YData = h.YData + pos(2);
        h.ZData = h.ZData + pos(3);
        view(UIAxes, az, el); % Restore view
        g_line3 = h;
    else
        g_line3 = [];
    end
    state_gripper_handles = [g_line1, g_line2, g_line3];
    
    % --- Plot Base ---
    plot3(UIAxes, 0, 0, 0, 'ks', 'MarkerSize', 15, 'MarkerFaceColor', 'k');
    
    % --- Set Plot Properties ---
    hold(UIAxes, 'off');
    axis(UIAxes, 'equal');
    grid(UIAxes, 'on');
    xlabel(UIAxes, 'X-axis (mm)');
    ylabel(UIAxes, 'Y-axis (mm)');
    zlabel(UIAxes, 'Z-axis (mm)');
    title(UIAxes, 'UR5 6-DOF Robot Arm');
    
    lim = 900; 
    axis(UIAxes, [-lim, lim, -lim, lim, -50, lim]);
    
    % Keep current view
    [az, el] = view(UIAxes);
    view(UIAxes, az, el);
end

% --- Helper to draw zones ---
function plot_zone(zone, color)
    v = [zone(1), zone(2), world_table_z+1;
         zone(3), zone(2), world_table_z+1;
         zone(3), zone(4), world_table_z+1;
         zone(1), zone(4), world_table_z+1];
    patch(UIAxes, 'Vertices', v, 'Faces', [1,2,3,4], ...
          'FaceColor', color, 'FaceAlpha', 0.2, 'EdgeColor', color, 'LineWidth', 1.5);
end

% --- Helper to enable/disable all controls (with E-Stop logic) ---
function set_controls_enabled(state_str)
    % state_str can be 'on', 'off', or 'e_stop'
    
    % Default states
    master_state = 'on';
    estop_reset_state = 'off';
    
    if strcmp(state_str, 'off') % Cycle running
        master_state = 'off';
    elseif strcmp(state_str, 'e_stop') % E-Stop active
        master_state = 'off';
        estop_reset_state = 'on';
    end
    
    % E-Stop buttons
    EStopButton.Enable = 'on'; % E-Stop is ALWAYS enabled
    ResetEStopButton.Enable = estop_reset_state;
    
    % Factory Tab
    SpawnPartsButton.Enable = master_state;
    ScanButton.Enable = master_state;
    RunCycleButton.Enable = master_state;
    OptimizeCheckbox.Enable = master_state;
    FailureSlider.Enable = master_state;
    NumPartsSlider.Enable = master_state;
    
    % Manual Tab
    for i = 1:robot.dof
        Sliders(i).Enable = master_state;
    end
    ResetButton.Enable = master_state;
    
    % IK Tab
    IK_X.Enable = master_state;
    IK_Y.Enable = master_state;
    IK_Z.Enable = master_state;
    IK_R.Enable = master_state;
    IK_P.Enable = master_state;
    IK_Y.Enable = master_state;
    GetPoseButton.Enable = master_state;
    SolveIKButton.Enable = master_state;
    
    % Task Tab
    OpenGripperButton.Enable = master_state;
    CloseGripperButton.Enable = master_state;
    SavePoseButton.Enable = master_state;
    RunPathButton.Enable = master_state;
    ClearPathButton.Enable = master_state;
    
    % View Panel
    IsoButton.Enable = master_state;
    TopButton.Enable = master_state;
    SideButton.Enable = master_state;
    CollisionCheck.Enable = master_state;
    
    drawnow; % Update GUI
end

% --- Helper to get position from Q (for retry planning) ---
function pos = pos_from_q(q_deg, offset)
    [T, ~] = compute_fk(robot, q_deg);
    pos = T(1:3, 4)' + offset;
end

end % --- END OF MAIN FUNCTION ---
