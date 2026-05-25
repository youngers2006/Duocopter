clc; 
clear;

% load data
data = load('sampleH.mat');
smplH = data.smplH;
time = smplH(1, :);
height = smplH(2, :);

% select endpoints with constant 
end_points = time > 68;
time = time(end_points);
height = height(end_points);

% Get noise variance
height_filtered = movmean(height, 20);
noise = height - height_filtered;
noise_variance = var(noise);

fprintf ('Noise Variance : %.3e\n', noise_variance);