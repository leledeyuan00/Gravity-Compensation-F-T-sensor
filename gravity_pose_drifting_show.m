clear
data = readmatrix("datas/GP2025-07-22-22-06-02.txt");
data = data(660:840,:);
time = data(:,1);
force = data(:,2:4);
torque = data(:,5:7);
quaternions = [data(:, 11), data(:, 8:10)];
euls = quat2eul(quaternions);
euls_norm = vecnorm(euls, 2,2);

force_norm = sqrt(sum(force.*force, 2));

%finds stoped timestamp
epsc = 1e-4;
stoped_times = [];
recorded_euls = [1];                        
duration = 20;

record_idx = false;
init_idx = 0;
count = 0;
for i = 1:length(euls_norm) - duration
    if abs(euls_norm(i) - euls_norm(i+1)) < epsc
        count = count+1;
        if ~record_idx
            record_idx = true;
            init_idx = i;
        end
    else
        count = 0;
        record_idx = false;
    end

    if count >=duration
        if abs(recorded_euls(end) - euls_norm(i-duration)) > epsc*10
            stoped_times(length(stoped_times)+1) = time(init_idx);
            recorded_euls(length(recorded_euls)+1) = euls_norm(init_idx);
            record_idx = false;
            count = 0;
        end
    end
end

font_size = 16;
f1 = figure();
yyaxis left
plot(time, force_norm, 'LineWidth', 1)
ylabel('Force Norm(N)', 'FontSize', font_size)

hold on
yyaxis right
plot(time, euls_norm, 'LineWidth',1)
ylim([3.5,8])
grid on
xlabel('Time(s)', 'FontSize',font_size)
ylabel('Eular Norm', 'FontSize', font_size)

xline(stoped_times, '--', 'Color', 'r', 'LineWidth', 1.0)

mean = mean(force_norm);
var = var(force_norm);
fprintf("Mean is: %f\n", mean);
fprintf("Var is: %f\n", var);