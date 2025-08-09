clear
data = readmatrix("datas/static_pose_recording.txt");
time = data(:,1);
force = data(:,2:4);
torque = data(:,5:7);

force_norm = sqrt(sum(force.*force, 2));

figure();
font_size = 16;
plot(time/3600, force_norm, 'LineWidth', 1)

grid on
xlabel('Time(h)', 'FontSize',font_size)
ylabel('Force Norm(N)', 'FontSize', font_size)

mean = mean(force_norm);
var = var(force_norm);
fprintf("Mean is: %f\n", mean);
fprintf("Var is: %f\n", var);