pose = readmatrix("datas/recorded_messages_pose.txt");
wrench = readmatrix("datas/recorded_messages_wrench.txt");

positions = pose(:,1:3)';
quaternions = [pose(:, 7), pose(:,4:6)]';

rots = quat2rotm(quaternions(1:4, :)');

force = wrench(:,1:3)';
torque = wrench(:,4:6)';


%% force
A = zeros(3*size(pose,1), 4);
b = zeros(3*size(pose,1), 1);
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
e = reshape(e, 3, []);
e_norm = vecnorm(e, 2, 1);
force_raw_norm = vecnorm(force, 2,1);

%% torque
M_xy_rev = [0,1,0;-1,0,0;0,0,0];
for i = 1:size(pose,1)
    index = (i-1)*3 + 1;
    R_t = zeros(3);
    R_t(:,3) = rots(:,3,i);
    % At1 = x(1) * M_xy_rev * R_t;
    At1 = M_xy_rev * R_t * [0;0;x(1)];
    At(index: index+2, 1) = At1;
    At(index: index+2, 2:4) = rots(:,:,i);
    bt(index: index+2, 1) = torque(:,i);
end

xt = inv(At'*At)*At'*bt;

pt = At * xt;
et = pt - bt;
et_norm = vecnorm(reshape(et, 3, []), 2, 1);
torque_raw_norm = vecnorm(torque, 2, 1);


fprintf("Estimated x_f: [%f, %f, %f, %f]\n", x(1), x(2), x(3), x(4));
fprintf("Estimated x_t: [%f, %f, %f, %f]\n", xt(1), xt(2), xt(3), xt(4));