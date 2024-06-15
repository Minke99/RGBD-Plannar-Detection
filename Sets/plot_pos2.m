%% 
clc;clear all;close all
%%% filtfilt Start%%%
fc=30;%cutoff frequency(Hz)
fs=1000;%sampling frequency(assumed)
[b,a] = butter(5,fc/(fs/2),'low');
%%% filtfilt End  %%%

%%
t = Abs_time;
x_diff = mean(x(1:10)-cam_px(1:10));
y_diff = mean(y(1:10)-cam_py(1:10));
z_diff = mean(z(1:10)-cam_pz(1:10));

vx = diff_extend(x, t, 6);
vy = diff_extend(y, t, 6);
vz = diff_extend(z, t, 6);

x_world = cam_px + x_diff;
y_world = cam_py + y_diff;
z_world = cam_pz + z_diff;


% vx_est = cam_inc_x(1:end-1)./diff(t);
% vy_est = cam_inc_y(1:end-1)./diff(t);
% vz_est = cam_inc_z(1:end-1)./diff(t);

dt = diff(t);



vx_est = cam_inc_x(1:end-1)./diff(t);
vy_est = cam_inc_y(1:end-1)./diff(t);
vz_est = cam_inc_z(1:end-1)./diff(t);


idx_vx = find(abs(vx_est)>3);
vx_est_st = vx_est;
vx_est_st(idx_vx) = nan;

idx_vy = find(abs(vy_est)>3);
vy_est_st = vy_est;
vy_est_st(idx_vy) = nan;

idx_vz = find(abs(vz_est)>3);
vz_est_st = vz_est;
vz_est_st(idx_vz) = nan;

z_filt = filtfilt(b,a,z_world);
%%
close all
figure()
subplot(3,2,1)
plot(vx,'b')
hold on
plot(vx_est_st, 'r')
plot(fused_vx, 'black')
ylim([-3,3])

subplot(3,2,3)
plot(vy,'b')
hold on
plot(vy_est_st, 'r')
plot(fused_vy, 'black')
ylim([-3,3])

% % 
subplot(3,2,5)
plot(vz,'b')
hold on
plot(vz_est_st, 'r')
plot(fused_vz, 'black')
ylim([-3,3])

subplot(3,2,2)
plot(x,'b')
hold on
plot(x_world, 'r')
plot(fused_x+ x_diff, 'black')

subplot(3,2,4)
plot(y,'b')
hold on
plot(y_world, 'r')
plot(fused_y+ y_diff, 'black')

subplot(3,2,6)
plot(z,'b')
hold on
plot(z_world, 'r')
plot(fused_z+ z_diff, 'black')


%%


%%
function vx = diff_extend(x,t,step)

temp_vx = (x(1 + step:end) - x(1:end - step))./(t(1 + step:end) - t(1:end - step));

temp1 = ones(1,step/2)* temp_vx(1);
temp2 = ones(1,step/2)* temp_vx(end);

vx = [temp1, temp_vx, temp2];

end