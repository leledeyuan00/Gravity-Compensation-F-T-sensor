clear
pose = readmatrix("recorded_messages_pose.txt");
wrench = readmatrix("recorded_messages_wrench.txt");

positions = pose(:,1:3)';
quaternions = pose(:,4:7)';

rots = quat2rotm(quaternions(1:4, :)');

force = wrench(:,1:3)';
torque = wrench(:,4:6)';

%%
% force_after = zeros(3,length(force));
% for i = 1:size(pose,1)
%     force_after(:,i) = rots(:,:,i) * force(:,i);
% end

%% force
for i = 1:size(pose,1)
    index = (i -1) * 3 + 1;
    A(index: index+2, 1:4) = zeros(3,4);
    A(index + 2, 1) = 1;
    A(index: index+2, 2:4) = rots(:,:,i);
    b(index:index+2,1) = force(:,i);
end


x = inv(A'*A)*A'*b;

p = A * x;

e = p - b;


%% torque
M_xy_rev = [0,1,0;-1,0,0;0,0,0];
for i = 1:size(pose,1)
    index = (i-1)*3 + 1;
    R_t = zeros(3);
    R_t(:,3) = rots(:,3,i);
    At1 = x(1) * M_xy_rev * R_t;
    At(index: index+2, 1) = At1(:,3);
    At(index: index+2, 2:4) = rots(:,:,i);
    bt(index: index+2, 1) = torque(:,i);
end

xt = inv(At'*At)*At'*bt;

pt = At * xt;
et = pt - bt;

%% Testing
f_gravity = [0,0,x(1)]';
f_drift = x(2:4);
t_drift = xt(2:4);

for i = 1:size(pose,1)
    
    A_comp = zeros(3,4);
    A_comp(3,1) = 1;
    A_comp(:,2:4) = rots(:,:,i);
    force_sensor(:,i) = A_comp * x - force(:,i);

    R_t = zeros(3);
    R_t(:,3) = rots(:,3,i);
    At1 = x(1) * M_xy_rev * R_t;
    At_comp(1:3, 1) = At1(:,3);
    At_comp(1:3, 2:4) = rots(:,:,i);

    torque_sensor(:,i) =  At_comp * xt - torque(:,i);
    
end


