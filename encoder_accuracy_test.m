%% generate open loop commands from goal 1st time 
% in mm and ms

close all;
dt = 1; % ms
time_int = 50; % (ms) needs to be inputted to teensy sketch

% riseSpeeds = [5 6 7 8 9]; for slow
riseSpeeds = [10 30 60 90 120];

% continuous velocity of the motors
Av = [];
for speed = riseSpeeds
    Av = [Av speed*ones(1,round(2000/speed + 250)) zeros(1, 500) -20*ones(1,250) zeros(1, 500)];
end

% discretise motor command
Av_disc = [];
for i = 1:time_int:numel(Av)
    Av_disc = [Av_disc Av(i)];
end

figure(1); hold on;
plot(Av);

% create a string to send to microcontroller
output = "M ";
for i = 1:length(Av_disc)
    output = append(output, sprintf('%.0f ' , Av_disc(i)));
end
disp(output);

%% processing recorded data and plotting position date
% open the mocap matlab file first

% close all;
encoder_scale = 30*pi/256; % encoder counts -> mm conversion
mocapFile = en_acc_fast0001; % insert name of mocap .mat file
% mocap data is already in mm

teensyTable = readtable('CoolTerm fast.txt'); % insert name of teensy serial recording txt
mocap_all_data = mocapFile.Trajectories.Labeled.Data; % add (1,:,:) at the end to select which label if there are multiple labeled trajectories
mocap_all_data = mocap_all_data(:,:,1:end-200); % cut off the end if needed

mocapX = squeeze(mocap_all_data(1,1,:));
mocapY = squeeze(mocap_all_data(1,2,:));
mocap1D = sqrt(mocapX.^2 + mocapY.^2);
mocap1D = mocap1D - mocap1D(1); % so it starts at 0
mocapZ = squeeze(mocap_all_data(1,3,:)); % the height (z location)
mocapZ = mocapZ - mocapZ(1); % calibrate start height, add an offset the first time to increase tension (- 220)

teensyArray = table2array(teensyTable);
motor_command = (teensyArray(:, 1));
encoder = encoder_scale*(teensyArray(:, 2));
encoder = -encoder + encoder(1);
teensyFrameRate = (teensyArray(:, 3));

% the number of ms between every data point
mocapSamplingInterval = 1000*1/mocapFile.FrameRate;
teensySamplingInterval = 1000/mean(teensyFrameRate);

% resample at every ms
motor_command = interp1(1:teensySamplingInterval:teensySamplingInterval*numel(motor_command), motor_command, 1:teensySamplingInterval*numel(motor_command)); 
encoder = interp1(1:teensySamplingInterval:teensySamplingInterval*numel(encoder), encoder, 1:teensySamplingInterval*numel(encoder)); 
mocap1D = interp1(1:mocapSamplingInterval:mocapSamplingInterval*numel(mocap1D), mocap1D, 1:mocapSamplingInterval*numel(mocap1D)); 
mocapZ = interp1(1:mocapSamplingInterval:mocapSamplingInterval*numel(mocapZ), mocapZ, 1:mocapSamplingInterval*numel(mocapZ));

% make them same length
encoder = encoder(1:numel(mocapZ));
motor_command = motor_command(1:numel(mocapZ));

set(groot,'defaultLineLineWidth', 1.3)
figure('Renderer', 'painters', 'Position', [10 10 900 500]); hold on;
title("Encoder Accuracy - Position", FontSize=13);
colororder({'black','black'})
yyaxis right
plot(motor_command, 'color', 'black')
ylabel("motor command (pwm)")
yyaxis left
plot(encoder, 'color', [114 147 203]./215)
plot(mocapZ, '-', 'color', [144 103 167]./255);
ylabel("distance (mm)", FontSize=12)
xlabel("t (ms)", FontSize=12);
legend( "encoder count","mocap location", "motor command")
set(gca,'fontsize', 14)

% for plotting the microcontrollers sampling rate:
figure('Renderer', 'painters', 'Position', [10 10 900 300]); hold on;
plot(1:teensySamplingInterval:teensySamplingInterval*numel(teensyFrameRate), teensyFrameRate); % move this to a plot below
xlabel("t (ms)", FontSize=12);
ylim([0 500])
ylabel({'Microcontroller';'Frame Rate (Hz)'});
set(gca,'fontsize', 14)

%% velocity

% data is now all in mm (scale matching with goal) and matched up at start time

% differentiate the position arrays for velocities (m/s as its mm/ms)
mocap1D_vel = diff(mocap1D);
mocapZ_vel = diff_w(mocapZ, 50);
encoder_vel = diff_w(encoder, 50);
encoder_vel = encoder_vel(1:numel(mocapZ_vel));
idx = isnan(mocapZ_vel);
Fit = polyfit(mocapZ_vel(~idx), encoder_vel(~idx),1); 

% % the errors from mocap 1D, mocap Z and each encoder
% error1D = posGoal - mocap1D(1:numel(posGoal));
% errorZ = - mocapZ(1,1:numel(posGoal)); % if its falling, increase both motors commands
% errorEnA = mocapZ(1:numel(posGoal)).*(posGoal - Encoder(1:numel(posGoal)));
% errorEnB = -mocapZ(1:numel(posGoal)).*(posGoal + Encoder2(1:numel(posGoal)));
% % errorEnA = diff(errorEnA);
% % errorEnB = diff(errorEnB);

close all;
figure('Renderer', 'painters', 'Position', [10 10 900 600]); hold on; grid on;
W = 15;
plot(filter(ones(1,W)/W,1,encoder_vel), 'color', [114 147 203]./215)
plot(mocapZ_vel, 'color', [144 103 167]./255)
legend("Encoder velocity", "mocap velocity");
title("Encoder Accuracy - Velocity", FontSize=13);
xlabel("t (ms)", FontSize=12);
ylabel("velocity (m/s)", FontSize=12)
set(gca,'fontsize', 14)

figure('Renderer', 'painters', 'Position', [10 10 900 600]); hold on; grid on;
title("Encoder velocity vs Mocap velocity")
ylabel("encoder velocity (m/s)")
xlabel("mocap velocity (m/s)")
plot(mocapZ_vel, encoder_vel, '.')
plot(min(mocapZ_vel):0.01:max(mocapZ_vel), (min(mocapZ_vel):0.01:max(mocapZ_vel)).*Fit(1)+Fit(2))
legend('',sprintf("Trend line: Y = %.2f X + %.2f", Fit(1), Fit(2)))
set(gca,'fontsize', 14)
