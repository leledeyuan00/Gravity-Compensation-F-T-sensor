function [Rse_ref, gb_ref, f0_ref, resnorm] = refine_lss(Fs_all, Rs_all, init)
% 多姿态 Levenberg‑Marquardt 精修
%
% 输入
%   Fs_all : Nx3          N 个静止姿态的力
%   Rs_all : 3x3xN        对应 R_eb
%   init   : struct with fields Rse, gb, f0  (来自 ftcal_grobner)
%
% 输出
%   Rse_ref, gb_ref, f0_ref  精修后参数
%   resnorm  最终残差

    x0 = [ rotm2eul(init.Rse), init.gb.', init.f0.' ];  % [1x9]
    lsfun = @(x) residuals(x, Fs_all, Rs_all);

    opts = optimoptions('lsqnonlin','Display','off');
    [x, resnorm] = lsqnonlin(lsfun, x0, [], [], opts);

    Rse_ref = eul2rotm(x(1:3));
    gb_ref  = x(4);
    f0_ref  = x(5:7).';
end

function r = residuals(x, Fs, Rs)
    Rse = eul2rotm(x(1:3));
    gb  = [0,0, x(4)]';
    f0  = x(5:7).';

    N = size(Fs,2);
    pred = zeros(3,N);
    for i = 1:N
        pred(:,i) = (Rse * (Rs(:,:,i) * gb)) + f0;
    end
    r = (Fs - pred);
    r = r(:);
end
