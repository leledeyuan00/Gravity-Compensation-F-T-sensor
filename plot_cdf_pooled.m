function  fig = plot_cdf_pooled(RMS_all, methods_all, keep_idx)

if nargin < 4; end

Ks = 3:11; K = numel(Ks);
M  = numel(keep_idx);

fig = figure('Color','w'); hold on; grid on
co = lines(M); set(gca,'ColorOrder',co,'NextPlot','replacechildren');

for im = 1:M
    j = keep_idx(im);
    vals = [];
    for ik = 1:K
        v = RMS_all{ik, j};
        vals = [vals, v(:).']; %#ok<AGROW>
    end
    vals = sort(vals(~isnan(vals)));
    n = numel(vals);
    F = (1:n) / n;
    plot(vals, F, 'LineWidth',1.3, 'DisplayName', methods_all{j});
    hold on;
end
xlim([0.5,5]);
xlabel('Residual RMS per element (N)');
ylabel('CDF'); ylim([0 1]);
title('CDF of residual RMS (pooled over k)');
legend('Location','southeast'); box on

end
