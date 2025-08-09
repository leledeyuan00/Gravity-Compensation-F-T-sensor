function [Rse, gb, f0, cost, rotation_err, Rse0, gb0, f00, duration] = ftcal_grobner(Fs, Rs)
    duration = 0;    
    start_time = toc;
    Fs_train = Fs(:,1:3);
    Rs_train = Rs(:,1:3);

    f1 = Fs(:,2) - Fs(:,1);
    f2 = Fs(:,3) - Fs(:,2);
    R1 = Rs(:,:,2) - Rs(:,:,1);
    R2 = Rs(:,:,3) - Rs(:,:,2);
    [a,b,c,d, g1, g2, g3] = grobner_solver(f1, f2, R1, R2);
    
    if_find_sol = false;
    best.err = inf;
    for k = 1:length(a)
        qk = double([a(k) b(k) c(k) d(k)]);
        if any(isnan(qk)),  continue,  end
        Rk  = quat2rotm(qk);
        gbk = double([g1(k); g2(k); g3(k)]);
        gbk_ef = zeros(3,3);
    
        for i = 1:size(Rs_train,3)
            gbk_ef(i,:) = Rk * Rs_train(:,:,i)' * gbk;
        end
    
        f0k = mean(Fs_train,1).' - mean(gbk_ef,2);
    
        res = Fs_train.' - (Rk * Rs_train(:,:,i)'* gbk)' - f0k.';
        err = norm(res(:));
    
        if err < best.err
            best.Rse = Rk; best.gb = gbk; best.f0 = f0k; best.err = err;
            if_find_sol = true;
        end
    end
    if if_find_sol
        Rse0 = best.Rse; gb0 = best.gb; f00 = best.f0;
    else
        % large number for make sure the data is wrong
        fprintf("Cannot solve current configuration\n");
        Rse=eul2rotm([3.0,1.0,2.0]);
        gb = [0,0,0]';
        f0 = [0,0,0]';
        cost = 1000;
        rotation_err = 1000;
        Rse0 = eul2rotm([3.0,1.0,2.0]);
        gb0 = [0,0,0]';
        f00 = [0,0,0]';
        return
    end
    duration = toc - start_time;
    %% refine with all N samples
    [Rse, gb, f0, cost] = refine_ls(Fs, Rs, struct('Rse',Rse0,'gb',gb0,'f0',f00));
    rotation_err = rad2deg(norm(rotm2eul(Rse)- rotm2eul(Rse0)));
    
    % fprintf('Rotation error (deg): %.3f\n', rad2deg(norm(rotm2eul(Rse)- rotm2eul(Rse0))));
    % fprintf('gb  error (norm):     %.4f\n', norm(gb-gb0));
    % fprintf('f0  error (norm):     %.4f\n', norm(f0-f00));
end

