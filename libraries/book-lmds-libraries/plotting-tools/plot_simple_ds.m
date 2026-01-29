function [fig1] = plot_simple_ds(ds_fun, target, ds_title, plot_range)
    fig1 = figure('Color', [1 1 1]);
    fig_limits = [target(1)-plot_range target(1)+plot_range target(2)-plot_range target(2)+plot_range];
    [hs] = plot_ds_model(fig1, ds_fun, [0 0]', fig_limits, 'low'); hold on;
    [ha] = scatter(target(1), target(2), 200, 'kd', 'filled'); hold on
    [ha] = text(target(1), target(2)-plot_range*0.20, '$x^*$', 'Interpreter', 'LaTex', 'FontSize', 20, 'FontWeight', 'bold', ...
        'BackgroundColor', [1 1 1], 'EdgeColor', [0 0 0]); hold on
    axis(fig_limits)
    box on
    grid on
    xlabel('$x_1$', 'Interpreter', 'LaTex', 'FontSize', 35);
    ylabel('$x_2$', 'Interpreter', 'LaTex', 'FontSize', 35);
    title(ds_title, 'Interpreter', 'LaTex', 'FontSize', 35);
end