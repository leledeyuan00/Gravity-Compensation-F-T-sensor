clear
% close all
pose = readmatrix("datas/recorded_messages_pose_100.txt");
wrench = readmatrix("datas/recorded_messages_wrench_100.txt");

train_num_inc = 9;

train_nums = 3:3+train_num_inc;
epochs = 30;

valid_es = cell(train_num_inc, 1);
valid_es_refine = cell(train_num_inc, 1);

valid_es_lsm = cell(train_num_inc,1);
valid_es_lsm_refine = cell(train_num_inc,1);

valid_es_lsms = cell(train_num_inc,1);
valid_es_lsms_refine = cell(train_num_inc,1);

rms_mean = cell(1,1); % train_num_inc * 6 methods
rms_noise = cell(1,1); % train_num_inc * 6 methods

eul_all = cell(train_num_inc,1); % record all of euls

gravities_m = zeros(train_num_inc, 6); % grobner, grobner_refine, lsm, lsm_refine, lsms, lsms_refine
gravities_v = zeros(train_num_inc,6);
success_all = zeros(train_num_inc,epochs);
success_rate = zeros(train_num_inc,1);

t_total = zeros(train_num_inc, 6);

for k = 1:train_num_inc
    train_num = train_nums(k);
    valid_num = length(pose) - train_num;

    valid_e_norms = zeros(1, valid_num * epochs);
    valid_e_norms_refine = zeros(1, valid_num * epochs);
    valid_e_norms_lsm = zeros(1, valid_num *epochs);
    valid_e_norms_lsm_refine = zeros(1, valid_num *epochs);
    valid_e_norms_lsms = zeros(1, valid_num *epochs);
    valid_e_norms_lsms_refine = zeros(1, valid_num *epochs);
    valid_residual_all = zeros(3*6, valid_num*epochs);

    eul_epoch = zeros(epochs, 3);

    gravities = zeros(epochs,6);

    fprintf("Train num is: %d\n", train_num)
    for epoch = 1:epochs
        
        train_idx = randperm(length(pose), train_num);
        valid_idx = setdiff(1:length(pose), train_idx);
       
        %% Train
        train_quaternions = [pose(train_idx,7), pose(train_idx,4:6)]'; % xyzw -> wxyz
        
        train_rots = quat2rotm(train_quaternions(1:4, :)');
        
        train_wrench = wrench(train_idx,:);
        train_force = train_wrench(:,1:3)';
        train_torque = train_wrench(:,4:6)';
        
        %% grobner method
        % force
        train_rots_grobner = zeros(3,3,train_num);
        train_force_grobner = zeros(3, train_num);
        for i = 1:train_num
            train_rots_grobner(:,:,i) = train_rots(:,:,i)';
            train_force_grobner(:,i) = train_rots_grobner(:,:,i) * train_force(:,i);
        end
        % time calculate
        tic
        [Rse, gb, f0, cost, r_err, Rse0, gb0, f00, t_grobner] = ftcal_grobner(train_force_grobner, train_rots_grobner);
        t_grobner_e = toc;
        
        roteul = norm(rotm2eul(Rse)/pi*180);
        if roteul > 200
            success_all(k,epoch) = false;
            continue;
        else
            success_all(k,epoch) = true;
            err_eular = [rotm2eul(train_rots_grobner(:,:,1));
                        rotm2eul(train_rots_grobner(:,:,2));
                        rotm2eul(train_rots_grobner(:,:,3))];
        end
        t_total(k, 1) = t_total(k, 1) + t_grobner;
        t_total(k, 2) = t_total(k, 2) + t_grobner_e;
        gravities(epoch,1) = norm(gb0);         
        gravities(epoch,2) = norm(gb);  
        eul_epoch(epoch,1) = norm(rotm2eul(Rse));

        %% LSM method
        % calculate time
        tic;
        A = zeros(3*train_num, 6);
        for i = 1:train_num
            index = (i -1) * 3 + 1;
            A(index:index + 2, 1:3) = eye(3);
            A(index: index+2, 4:6) = train_rots(:,:,i);
            b(index:index+2,1) = train_force(:,i);
        end
          
        train_x = inv(A'*A)*(A'*b);
        t_lsm_e = toc;
        [Rse_lsm, gb_lsm, f0_lsm, resnorm_lsm] = refine_ls( train_force_grobner, train_rots_grobner, struct('Rse',eye(3),'gb',train_x(1:3,1),'f0',train_x(4:6,1)));
        t_lsmr_e = toc;
        t_total(k, 3) = t_total(k, 3) + t_lsm_e;
        t_total(k, 4) = t_total(k, 4) + t_lsmr_e;

        gravities(epoch,3) = norm(train_x(1:3)); 
        gravities(epoch,4) = norm(gb_lsm); 
        eul_epoch(epoch,2) = norm(rotm2eul(Rse_lsm));

        % LSMS
        % calculate time
        tic;
        As = zeros(3*train_num, 4);
        for i = 1:train_num
            index = (i -1) * 3 + 1;
            As(index + 2, 1) = 1;
            As(index: index+2, 2:4) = train_rots(:,:,i);
            bs(index:index+2,1) = train_force(:,i);
        end
          
        train_sx = inv(As'*As)*(As'*bs);
        t_lsms_e = toc;


        [Rse_lsms, gb_lsms, f0_lsms, resnorm_lsms] = refine_lss( train_force_grobner, train_rots_grobner, struct('Rse',eye(3),'gb',train_sx(1,1),'f0',train_sx(2:4,1)));
        t_lsmsr_e = toc;

        t_total(k, 5) = t_total(k, 5) + t_lsms_e;
        t_total(k, 6) = t_total(k, 6) + t_lsmsr_e;
        gravities(epoch,5) = train_sx(1);
        gravities(epoch,6) = norm(gb_lsms);
        eul_epoch(epoch,3) = norm(rotm2eul(Rse_lsms));

        %% Valid
        valid_positions = pose(valid_idx,1:3)';
        valid_quaternions = [pose(valid_idx,7), pose(valid_idx,4:6)]'; % xyzw -> wxyz
        
        valid_rots = quat2rotm(valid_quaternions(1:4, :)');
        
        valid_wrench = wrench(valid_idx,:);
        valid_force = valid_wrench(:,1:3)';
        valid_torque = valid_wrench(:,4:6)';
        valid_e_norm = zeros(1, valid_num);
        valid_e_norm_refine = zeros(1, valid_num);
        valid_e_norm_lsm_refine = zeros(1, valid_num);
        valid_e_norm_lsms_refine = zeros(1, valid_num);
        valid_rots_grobner = zeros(3,3,valid_num);
        valid_force_grobner = zeros(3, valid_num);

        valid_residual = zeros(3*6, valid_num);
        for i = 1:valid_num
            valid_rots_grobner(:,:,i) = valid_rots(:,:,i)';
            valid_force_grobner(:,i) = valid_rots_grobner(:,:,i) * valid_force(:,i);
            % grobner
            pred = (Rse0 * (valid_rots_grobner(:,:,i) * gb0)) + f00;
            valid_e_norm(1,i) = norm( pred- valid_force_grobner(:,i));
            valid_residual(1:3, i) = pred- valid_force_grobner(:,i);

            % grobner refine
            pred_refine = (Rse * (valid_rots_grobner(:,:,i) * gb)) + f0;
            valid_e_norm_refine(1, i) = norm( pred_refine - valid_force_grobner(:,i));
            valid_residual(4:6, i) = pred_refine- valid_force_grobner(:,i);
            
            % lsm refine
            pred_lsm = (Rse_lsm * (valid_rots_grobner(:,:,i) * gb_lsm)) + f0_lsm;
            valid_e_norm_lsm_refine(1, i) = norm( pred_lsm - valid_force_grobner(:,i));
            valid_residual(10:12, i) = pred_lsm- valid_force_grobner(:,i);

            % lsms refine
            pred_lsms = (Rse_lsms * (valid_rots_grobner(:,:,i) * [0,0,gb_lsms]')) + f0_lsms;
            valid_e_norm_lsms_refine(1, i) = norm( pred_lsms - valid_force_grobner(:,i));
            valid_residual(16:18, i) = pred_lsms- valid_force_grobner(:,i);
            
        end
                
        %% LSM method
        valid_A = zeros(3*valid_num,6);
        valid_b = zeros(3*valid_num, 1);
        for i = 1:valid_num
            index = (i -1) * 3 + 1;
            valid_A(index:index + 2, 1:3) = eye(3);
            valid_A(index: index+2, 4:6) = valid_rots(:,:,i);
            valid_b(index:index+2,1) = valid_force(:,i);
        end
        
        valid_p = valid_A * train_x;
        valid_e = valid_p - valid_b;
        valid_e_reshape = reshape(valid_e, [3, valid_num]);
        valid_e_norm_lsm = vecnorm(valid_e_reshape,2);
        valid_residual(7:9, :) = valid_e_reshape;

        % LSMS
        valid_As = zeros(3*valid_num,4);
        valid_bs = zeros(3*valid_num, 1);
        for i = 1:valid_num
            index = (i -1) * 3 + 1;
            valid_As(index + 2, 1) = 1;
            valid_As(index: index+2, 2:4) = valid_rots(:,:,i);
            valid_bs(index:index+2,1) = valid_force(:,i);
        end
        
        valid_ps = valid_As * train_sx;
        valid_lsms_e = valid_ps - valid_bs;
        valid_es_reshape = reshape(valid_lsms_e, [3, valid_num]);
        valid_e_norm_lsms = vecnorm(valid_es_reshape,2);
        valid_residual(13:15, :) = valid_es_reshape;

        %% record successful datas
        valid_index_start = (epoch -1) * valid_num + 1;
        valid_e_norms(1, valid_index_start: valid_index_start+valid_num-1) = valid_e_norm;
        valid_e_norms_refine(1, valid_index_start: valid_index_start+valid_num-1) = valid_e_norm_refine;
        valid_e_norms_lsm(1, valid_index_start: valid_index_start+valid_num-1) = valid_e_norm_lsm;
        valid_e_norms_lsm_refine(1, valid_index_start: valid_index_start+valid_num-1) = valid_e_norm_lsm_refine;
        valid_e_norms_lsms(1, valid_index_start: valid_index_start+valid_num-1) = valid_e_norm_lsms;
        valid_e_norms_lsms_refine(1, valid_index_start: valid_index_start+valid_num-1) = valid_e_norm_lsms_refine;
        
        valid_residual_all(:, valid_index_start: valid_index_start+valid_num-1) = valid_residual;

    end
    % Only use success == 1 elm
    suc_epoch = find(success_all(k,:) == 1);
    success_count = length(suc_epoch);
    valid_e_norms_suc = zeros(1, success_count*valid_num);
    valid_e_norms_refine_suc = zeros(1, success_count*valid_num);
    valid_e_norms_lsm_suc = zeros(1, success_count*valid_num);
    valid_e_norms_lsm_refine_suc = zeros(1, success_count*valid_num);
    valid_e_norms_lsms_suc = zeros(1, success_count*valid_num);
    valid_e_norms_lsms_refine_suc = zeros(1, success_count*valid_num);
    gravities_suc = zeros(success_count, 6);
    eul_suc = zeros(success_count, 3);
    valid_residual_all_suc = zeros(3*6, success_count*valid_num);

    for i = 1:success_count
        index_start = (suc_epoch(i) -1) * valid_num + 1;
        valid_e_norms_suc(1, (i-1)*valid_num+1 : i*valid_num) = valid_e_norms(index_start: index_start+valid_num-1);
        valid_e_norms_refine_suc(1, (i-1)*valid_num+1 : i*valid_num) = valid_e_norms_refine(index_start: index_start+valid_num-1);
        valid_e_norms_lsm_suc(1, (i-1)*valid_num+1 : i*valid_num) = valid_e_norms_lsm(index_start: index_start+valid_num-1);
        valid_e_norms_lsm_refine_suc(1, (i-1)*valid_num+1 : i*valid_num) = valid_e_norms_lsm_refine(index_start: index_start+valid_num-1);
        valid_e_norms_lsms_suc(1, (i-1)*valid_num+1 : i*valid_num) = valid_e_norms_lsms(index_start: index_start+valid_num-1);
        valid_e_norms_lsms_refine_suc(1, (i-1)*valid_num+1 : i*valid_num) = valid_e_norms_lsms_refine(index_start: index_start+valid_num-1);

        gravities_suc(i,:) = gravities(suc_epoch(i), :);
        eul_suc(i, :) = eul_epoch(suc_epoch(i), :);
        
        valid_residual_all_suc(:, (i-1)*valid_num+1 : i*valid_num) = valid_residual_all(:, index_start: index_start+valid_num-1);
    end

    %RMS
    valid_es{k} =               rms(reshape(valid_e_norms_suc', valid_num, []));    
    valid_es_refine{k} =        rms(reshape(valid_e_norms_refine_suc', valid_num, []));    
    valid_es_lsm{k} =           rms(reshape(valid_e_norms_lsm_suc', valid_num, []));
    valid_es_lsm_refine{k} =    rms(reshape(valid_e_norms_lsm_refine_suc', valid_num, []));
    valid_es_lsms{k} =          rms(reshape(valid_e_norms_lsms_suc', valid_num, []));
    valid_es_lsms_refine{k} =   rms(reshape(valid_e_norms_lsms_refine_suc', valid_num, []));

    gravities_m(k,:) = mean(gravities_suc,1);
    gravities_v(k,:) = var(gravities_suc,0,1);
    eul_all{k} = eul_suc;
    

    mu = zeros(3*6, success_count);
    noise = zeros(6, success_count);

    for i = 1:success_count % successed epochs
        index_start = (i -1 ) * valid_num +1;
        index_end = i * valid_num;
        
        % valid_residual_all_suc : [3*6, epoch*valid_nums]
        mu_epoch = mean(valid_residual_all_suc(:, index_start: index_end), 2);
        mu(:, i) = mu_epoch;
        noise(:, i) = sqrt(mean(reshape(vecnorm(...
                        reshape(...
                            (valid_residual_all_suc(:, index_start: index_end) - mu_epoch), 3, []))...
                            , 6, []).^2, 2) ...
                            );
    end
    for i = 1:6
        rms_mean{k, i} = mu((i-1)*3+1: i*3, : );
        rms_noise{k, i } = noise(i, :);
    end
    
    fprintf("Mean of valid: %f\n", mean(valid_e_norms_refine_suc))
    fprintf("Var of valid: %f\n", var(valid_e_norms_refine_suc))
    success_rate(k) = success_count / length(success_all);
end

%% Plot
cmap = parula(train_num_inc); 
alphaVal = 0.65;

% Grobner Initial
figure('Color','w'); box on; hold on
for g = 1:train_num_inc
    boxchart( repelem(g, size(valid_es{g},2))', valid_es{g}(1,:)','BoxFaceColor',cmap(1,:), ...
             'BoxFaceAlpha',alphaVal, ...
             'WhiskerLineColor',[0.3 0.3 0.3], ...
             'LineWidth',1.1, ...
             'MarkerStyle','none');  
end


xticks(1:train_num_inc); xticklabels(compose("k=%d",3:3+train_num_inc-1))

set(gca,'FontSize',12)
title("Grobner Estimated Initial Parameters");
grid on;

% Grobner with refine
figure('Color','w'); box on; hold on
for g = 1:train_num_inc
    boxchart( repelem(g, size(valid_es_refine{g},2))', valid_es_refine{g}(1,:)','BoxFaceColor',cmap(2,:), ...
             'BoxFaceAlpha',alphaVal, ...
             'WhiskerLineColor',[0.3 0.3 0.3], ...
             'LineWidth',1.1, ...
             'MarkerStyle','none'); 
end

xticks(1:train_num_inc); xticklabels(compose("k=%d",3:3+train_num_inc-1))
ylim([0,1.9])
set(gca,'FontSize',12)
title("Refined Grobner Estimated Initial Parameters");
grid on;

% LSM-FULL Initial
figure('Color','w'); box on; hold on
for g = 1:train_num_inc
    boxchart( repelem(g, size(valid_es_lsm{g},2))', valid_es_lsm{g}(1,:)','BoxFaceColor',cmap(3,:), ...
             'BoxFaceAlpha',alphaVal, ...
             'WhiskerLineColor',[0.3 0.3 0.3], ...
             'LineWidth',1.1, ...
             'MarkerStyle','none'); 
end

xticks(1:train_num_inc); xticklabels(compose("k=%d",3:3+train_num_inc-1))
ylim([0,1.9])
set(gca,'FontSize',12)
title("LSM-full Estimated Initial Parameters");
grid on;


% LSM-FULL Refine
figure('Color','w'); box on; hold on
for g = 1:train_num_inc
    boxchart( repelem(g, size(valid_es_lsm_refine{g},2))', valid_es_lsm_refine{g}(1,:)','BoxFaceColor',cmap(4,:), ...
             'BoxFaceAlpha',alphaVal, ...
             'WhiskerLineColor',[0.3 0.3 0.3], ...
             'LineWidth',1.1, ...
             'MarkerStyle','none');  
end

xticks(1:train_num_inc); xticklabels(compose("k=%d",3:3+train_num_inc-1))
ylim([0,1.9])
set(gca,'FontSize',12)
title("LSM-full Refined Estimated Parameters");
grid on;

% LSM-short Initial
figure('Color','w'); box on; hold on
for g = 1:train_num_inc
    boxchart( repelem(g, size(valid_es_lsms{g},2))', valid_es_lsms{g}(1,:)','BoxFaceColor',cmap(5,:), ...
             'BoxFaceAlpha',alphaVal, ...
             'WhiskerLineColor',[0.3 0.3 0.3], ...
             'LineWidth',1.1, ...
             'MarkerStyle','none'); 
end

xticks(1:train_num_inc); xticklabels(compose("k=%d",3:3+train_num_inc-1))
ylim([0,1.9])
set(gca,'FontSize',12)
title("LSM-short Estimated Initial Parameters");
grid on;

% LSM-FULL Refined
figure('Color','w'); box on; hold on
for g = 1:train_num_inc
    boxchart( repelem(g, size(valid_es_lsms_refine{g},2))', valid_es_lsms_refine{g}(1,:)','BoxFaceColor',cmap(6,:), ...
             'BoxFaceAlpha',alphaVal, ...
             'WhiskerLineColor',[0.3 0.3 0.3], ...
             'LineWidth',1.1, ...
             'MarkerStyle','none'); 
end

xticks(1:train_num_inc); xticklabels(compose("k=%d",3:3+train_num_inc-1))
ylim([0,1.9])
set(gca,'FontSize',12)
title("Refined LSM-short Estimated Initial Parameters");
grid on;


%% Ftest for LSM short and LSM full
RSS_lsm = valid_es_lsm{9};
RSS_lsm = RSS_lsm.*RSS_lsm * length(RSS_lsm);
RSS_lsms = valid_es_lsms{9};
RSS_lsms = RSS_lsms.*RSS_lsms* length(RSS_lsms);

Fvals = ((RSS_lsms - RSS_lsm)/ (6-4))./ ...
        (RSS_lsm./ (length(RSS_lsm) - 6)) ;

pvals = 1- fcdf(Fvals, 2, length(RSS_lsm) - 6);
fprintf('Mean p(F-test) = %.4g\n', mean(pvals));
mean((RSS_lsms - RSS_lsm)./ (RSS_lsms) * 100)

success_count_t = zeros(train_num_inc,1);
for i = 1:train_num_inc
    success_count_t(i) = length(find(success_all(i,:) == 1));
    t_total_mean(i, :) = t_total(i, :) / success_count_t(i);
end


%% Process all of the data for plot
% RMS
RMS_all = cell(1);
for i = 1:train_num_inc
    RMS_all{i,1} = valid_es{i};
    RMS_all{i,2} = valid_es_refine{i};
    RMS_all{i,3} = valid_es_lsm{i};
    RMS_all{i,4} = valid_es_lsm_refine{i};
    RMS_all{i,5} = valid_es_lsms{i};
    RMS_all{i,6} = valid_es_lsms_refine{i};
end

% RSS
RSS_all = cell(1);
for i = 1:train_num_inc
    for j = 1:6
        RSM = RMS_all{i,j};
        RSS_all{i,j} = RSM.*RSM*length(RSM);
    end
end

% ||u||
rms_mean;
rms_u= cell(1);
for i = 1:train_num_inc
    for j = 1:6
        rms_u{i,j} = vecnorm(rms_mean{i,j}, 2, 1);
    end
end
% RMS_Noise
rms_noise;

rms_noise_7 = zeros(6,1);
for i= 1:6
    rms_noise_7(i) = mean(rms_noise{8,i});
end

%%
methods_all = {'Grobner','Grobner_ref','LSM-full','LSM-full_ref','LSM-short','LSM-short_ref'};
keep_idx = [3 4 5 6];   % 忽略 Grobner 未 refine

f1 = plot_cdf_pooled(RMS_all, methods_all, keep_idx);