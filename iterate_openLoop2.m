%% generate open loop commands from goal 1st time
% generate 0.2 Hz sine wave 
% in mm and ms

close all;
dt = 1; % ms
time_int = 50; % ms (command every 0.5s) need to be inputted to teensy sketch
% total_time = 6001;

% for the step response
% dist = 500;
% posGoal = [0*ones(1, 1000) 0:dist/500:dist dist*ones(1, 2000) dist:-dist/500:0 0*ones(1, 1000)]; % for trapezium (step response)

% for the sine waves
amplitude = 150;
a = 0:2000;
b = 1000:2000;

posGoal = [amplitude*cos(2*pi*a/1000)];
% [amplitude*cos(2*pi*a/2000)]
%[amplitude*cos(2*pi*a/500)] 
%    [amplitude*cos(2*pi*b/250)];
% 
  
% continuous velocity of the motors
Av = diff(posGoal)/dt; 
Bv = diff(-posGoal)/dt;

% scale from m/s to motor command levels (-255 to 255)
% initially by an arbitrary value, it will be adjusted each iteration
Av_initialCommand = Av * 6 + (Av >= 0).*Av*20 + 3;
Bv_initialCommand = Bv * 6 + (Bv >= 0).*Bv*20 + 3;

Av_disc = [];
Bv_disc = [];

% discretise motor command
for i = 1:time_int:numel(Av_initialCommand)
    Av_disc = [Av_disc Av_initialCommand(i)];
    Bv_disc = [Bv_disc Bv_initialCommand(i)];
end

figure(1); hold on;
plot(posGoal)
plot(Av);
plot(Bv);
% plot(Av_initialCommand);
% plot(Bv_initialCommand);

output = "M ";
for i = 1:length(Av_disc)
    output = append(output, sprintf('%.0f ' , Av_disc(i)));
    output = append(output, sprintf('%.0f ' , Bv_disc(i)));
end
disp(output);

%% processing recorded data and plotting position date

% open the mocap matlab file first

close all;

encoder_scale = 30*pi/256; % counts to mm conversion
mocapFile = OL_sine_slowTwo0007; % change name of mocap mat file each time
% mocap data is already in mm

teensyTable = readtable('CoolTerm_sine_slow_2nd.txt'); % insert name of teensy serial file
mocap_all_data = mocapFile.Trajectories.Labeled.Data; % insert name of mocap file % add (1,:,:) at the end to select which label
% mocap_all_data = mocap_all_data(:,:,1:760); % cut off the end

mocapX = squeeze(mocap_all_data(1,1,:));
mocapY = squeeze(mocap_all_data(1,2,:));
mocap1D = sqrt(mocapX.^2 + mocapY.^2);
mocap1D = mocap1D - mocap1D(1); % so it starts at 0
mocapZ = squeeze(mocap_all_data(1,3,:)); % the height (z location)
mocapZ = mocapZ - mocapZ(1); % calibrate start height, add an offset the first time to increase tension (- 220)

teensyArray = table2array(teensyTable);
Encoder1 = encoder_scale*(teensyArray(:, 2));
Encoder1 = Encoder1 - Encoder1(1);
Encoder2 = encoder_scale*(teensyArray(:, 4));
Encoder2 = Encoder2 - Encoder2(1);
M1_command = 1*(teensyArray(:, 1));
M2_command = 1*(teensyArray(:, 3));
teensyFrameRate = (teensyArray(:, 5));

% the number of ms between every data point
mocapSamplingInterval = 1000*1/mocapFile.FrameRate;
teensySamplingInterval = 1000/mean(teensyFrameRate);

% resample at every ms (like posGoal) so can differentiate and compute
% error later
Encoder1 = interp1(1:teensySamplingInterval:teensySamplingInterval*numel(Encoder1), Encoder1, 1:teensySamplingInterval*numel(Encoder1)); 
Encoder2 = interp1(1:teensySamplingInterval:teensySamplingInterval*numel(Encoder2), Encoder2, 1:teensySamplingInterval*numel(Encoder2)); 
mocap1D = interp1(1:mocapSamplingInterval:mocapSamplingInterval*numel(mocap1D), mocap1D, 1:mocapSamplingInterval*numel(mocap1D)); 
mocapZ = interp1(1:mocapSamplingInterval:mocapSamplingInterval*numel(mocapZ), mocapZ, 1:mocapSamplingInterval*numel(mocapZ));

% remove any NaN values that are created at the end of the interpolation
mocap1D = mocap1D(~isnan(mocap1D));
mocapZ = mocapZ(~isnan(mocapZ));
mocap1D = mocap1D(1,1:numel(Encoder1));
mocapZ = mocapZ(1,1:numel(Encoder1));

set(groot,'defaultLineLineWidth', 1.3)
figure('Renderer', 'painters', 'Position', [10 10 900 500]); hold on;
title("0.5 Hz and 1 Hz Sine Wave Position Data (initial command)", FontSize=13);
plot(-posGoal, 'color', 'black')
plot(Encoder1, 'color', [114 147 203]./215)
plot(Encoder2, 'color', [211 94 96]./215)
plot(mocapZ, 'color', [144 103 167]./255);
plot(mocap1D, 'color', "#EDB120");
ylabel("distance (mm)", FontSize=12)
xlabel("t (ms)", FontSize=12);
legend("Goal", "Encoder 1 reading", "Encoder 2 reading","Z location (mocap)", "1D location (mocap)")
set(gca,'fontsize', 14)

% % for plotting the microcontrollers sampling rate:
% figure('Renderer', 'painters', 'Position', [10 10 900 300]); hold on;
% plot(1:teensySamplingInterval:teensySamplingInterval*numel(teensyFrameRate), teensyFrameRate); % move this to a plot below
% xlabel("t (ms)", FontSize=12);
% ylim([0 500])
% ylabel({'Microcontroller';'Frame Rate (Hz)'});
% set(gca,'fontsize', 14)


%% plotting velocity data and updating driving signal

% data is now all in mm (scale matching with goal) and matched up at start time

% update these to previous command (continuous) as generated each iteration
Av_old = Av_initialCommand;
Bv_old = Bv_initialCommand;

% differentiate the position arrays for velocities (m/s as its mm/ms)
mocap1D_vel = diff(mocap1D);
% En1_vel = diff(Encoder1);
% En2_vel = diff(Encoder2);
mocap1D_vel = mocap1D_vel(1, 1:numel(Av_old));
mocapZ = mocapZ(1, 1:numel(Av_initialCommand));

% gains for the different errors (tweak)
gain1D = 4.0;
gainZ = 0.05;
gainEn = 1;

% the errors from mocap 1D, mocap Z and each encoder
error1D = Av - mocap1D_vel;
errorZ = - mocapZ; % if its falling, increase both motors commands
errorEnA = posGoal - Encoder1(1:numel(posGoal));
errorEnB = -posGoal - Encoder2(1:numel(posGoal));
errorEnA = diff(errorEnA);
errorEnB = diff(errorEnB);

% updates for each motor
updateA = [error1D*gain1D zeros(1, numel(Av_old)-numel(error1D))] + [errorZ*gainZ zeros(1, numel(Av_old)-numel(errorZ))] + errorEnA*gainEn;
updateB = [-error1D*gain1D zeros(1, numel(Av_old)-numel(error1D))] + [errorZ*gainZ zeros(1, numel(Bv_old)-numel(errorZ))] + errorEnB*gainEn;

% new motor command sequences (each ms)
Av_new = Av_old + updateA;
Bv_new = Bv_old + updateB;

% limit maximum and minimum values
Av_new(Av_new > 127) = 127;
Av_new(Av_new < -127) = -127;
Bv_new(Bv_new > 127) = 127;
Bv_new(Bv_new < -127) = -127;

Av_new_disc = [];
Bv_new_disc = [];

% discretise motor command by averaging each chuck
for i = time_int+1:time_int:numel(posGoal)
    avg_A = mean(Av_new(i-time_int:i));
    avg_B = mean(Bv_new(i-time_int:i));
    Av_new_disc = [Av_new_disc avg_A];
    Bv_new_disc = [Bv_new_disc avg_B];
end

% create string to send to teensy with the new motor commands
output = "M ";
for i = 1:length(Av_new_disc)
    output = append(output, sprintf('%.0f ' , Av_new_disc(i)));
    output = append(output, sprintf('%.0f ' , Bv_new_disc(i)));
end
disp(output);

close all;
figure('Renderer', 'painters', 'Position', [10 10 900 600]); hold on;
plot(Av_new, 'color', [57 106 177]./255)
plot(Bv_new, 'color', [204 37 41]./255)
plot(Av_old, ':', 'color', [57 106 177]./255)
plot(Bv_old, ':', 'color', [204 37 41]./255)
plot(errorEnA*gainEn, 'color', [114 147 203]./215)
plot(errorEnB*gainEn, 'color', [211 94 96]./215)
plot(error1D*gain1D, 'color', "#EDB120")
plot(errorZ*gainZ, 'color', [144 103 167]./255)
legend("Motor 1 new", "Motor 2 new", "Motor 1 old", "Motor 2 old", "Encoder 1 error adjustment", "Encoder 2 error adjustment","mocap 1D error adjustment", "mocap Z error adjustment");
title("Step Response Velocity Adjustments (2nd iteration)", FontSize=13);
xlabel("t (ms)", FontSize=12);
ylabel("velocity (m/s)", FontSize=12)
set(gca,'fontsize', 14) 

% the unit is m/s but the errors are multiplied by a gain


% just for checking, actually plot separately with all the command
% iterations
figure(3); hold on;
plot(Av_new_disc);
plot(Bv_new_disc);



