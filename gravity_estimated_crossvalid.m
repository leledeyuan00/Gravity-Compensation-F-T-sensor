clear
pose = readmatrix("datas/recorded_messages_pose_100.txt");
wrench = readmatrix("datas/recorded_messages_wrench_100.txt");

train_num_inc = 9;

train_nums = 3:3+train_num_inc;
epochs = 30;
train_es = cell(train_num_inc, 1);
valid_es = cell(train_num_inc, 1);

gravities = zeros(train_num_inc, epochs);

for k = 1:train_num_inc
    train_num = train_nums(k);
    valid_num = length(pose) - train_num;

    train_e_norms = zeros(1, train_num * epochs);
    valid_e_norms = zeros(1, valid_num * epochs);

    fprintf("Train num is: %d\n", train_num)
    for epoch = 1:epochs
        
        train_idx = randperm(length(pose), train_num);
        valid_idx = setdiff(1:length(pose), train_idx);
       
        % Train
        train_positions = pose(train_idx,1:3)';
        train_quaternions = [pose(train_idx,7), pose(train_idx,4:6)]'; % xyzw -> wxyz
        
        train_rots = quat2rotm(train_quaternions(1:4, :)');
        
        train_wrench = wrench(train_idx,:);
        train_force = train_wrench(:,1:3)';
        train_torque = train_wrench(:,4:6)';
        
        % force
        A = zeros(3*train_num, 4);
        for i = 1:size(train_positions,2)
            index = (i -1) * 3 + 1;
            A(index + 2, 1) = 1;
            A(index: index+2, 2:4) = train_rots(:,:,i);
            b(index:index+2,1) = train_force(:,i);
        end
          
        train_x = inv(A'*A)*(A'*b);
        train_p = A * train_x;
        train_e = train_p - b;
        train_e_rashape = reshape(train_e, [3, train_num]);
        train_e_norm = vecnorm(train_e_rashape,2);
        
        train_index_start = (epoch -1) * train_num + 1;
        train_e_norms(1, train_index_start: train_index_start+train_num-1) = train_e_norm;
        
        gravities(k,epoch) = train_x(1);  
        
        % torque
        M_xy_rev = [0,1,0;-1,0,0;0,0,0];
        for i = 1:size(train_positions,2)
            index = (i-1)*3 + 1;
            R_t = zeros(3);
            R_t(:,3) = train_rots(:,3,i);
            At1 = train_x(1) * M_xy_rev * R_t;
            At(index: index+2, 1) = At1(:,3);
            At(index: index+2, 2:4) = train_rots(:,:,i);
            bt(index: index+2, 1) = train_torque(:,i);
        end
        
        xt = inv(At'*At)*At'*bt;
        
        pt = At * xt;
        et = pt - bt;
        
        % Valid
        valid_positions = pose(valid_idx,1:3)';
        valid_quaternions = [pose(valid_idx,7), pose(valid_idx,4:6)]'; % xyzw -> wxyz
        
        valid_rots = quat2rotm(valid_quaternions(1:4, :)');
        
        valid_wrench = wrench(valid_idx,:);
        valid_force = valid_wrench(:,1:3)';
        valid_torque = valid_wrench(:,4:6)';
        
        valid_A = zeros(3*valid_num,4);
        valid_b = zeros(3*valid_num, 1);
        for i = 1:size(valid_positions,2)
            index = (i -1) * 3 + 1;
            valid_A(index + 2, 1) = 1;
            valid_A(index: index+2, 2:4) = valid_rots(:,:,i);
            valid_b(index:index+2,1) = valid_force(:,i);
        end
        
        valid_p = valid_A * train_x;
        valid_e = valid_p - valid_b;
        valid_e_rashape = reshape(valid_e, [3, valid_num]);
        valid_e_norm = vecnorm(valid_e_rashape,2);
        
        valid_index_start = (epoch -1) * valid_num + 1;
        valid_e_norms(1, valid_index_start: valid_index_start+valid_num-1) = valid_e_norm;
    end
    train_es{k} = train_e_norms;
    valid_es{k} = valid_e_norms;

    fprintf("Mean of train: %f\n", mean(train_e_norms))
    fprintf("Var of train: %f\n", var(train_e_norms))
    
    
    fprintf("Mean of valid: %f\n", mean(valid_e_norms))
    fprintf("Var of valid: %f\n", var(valid_e_norms))
end

%% Plot

cmap = parula(train_num_inc); 
alphaVal = 0.65;        

figure('Color','w'); box on; hold on
for g = 1:train_num_inc
    boxchart( repelem(g, size(valid_es{g},2))', valid_es{g}(1,:)','BoxFaceColor',cmap(g,:), ...
             'BoxFaceAlpha',alphaVal, ...
             'WhiskerLineColor',[0.3 0.3 0.3], ...
             'LineWidth',1.1, ...
             'MarkerStyle','none');  
end

xticks(1:train_num_inc); xticklabels(compose("k=%d",3:3+train_num_inc-1))
ylim([0,1.9])
set(gca,'FontSize',12)
grid on;
