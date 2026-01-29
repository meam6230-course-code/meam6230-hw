function plot_ds_modulation(x, y, x_dot, y_dot, x0, titleName, type, target, plot_speed)
% PLOT_DS  Plot a dynamical system on a grid.
%   PLOT_DS(X, Y, X_DOT, Y_DOT, X0, TITLENAME, TYPE, X_TARGET) where the 
%   arrays X,Y define the coordinates for X_DOT,Y_DOT and are monotonic and
%   2-D plaid (as if produced by MESHGRID), plots a dynamical system with
%   attractor(s) X0 given as 2xN vector data and name TITLENAME.
%
%   The variable TYPE is one of 'streamslice', 'streamline' and defines the
%   plotting style of the DS.
%
%   The optional variable X_TARGET given as 2xN vector data can be used to
%   plot additional points of interest (e.g. local modulation points).
    
    title(titleName, 'Interpreter','latex', 'FontSize',15)
    if plot_speed
        [~, h] = contourf(x, y, sqrt(x_dot.^2 + y_dot.^2), 80);
        set(h, 'LineColor', 'none'); 
        colormap('summer');
        c_bar = colorbar;
        c_bar.Label.String = 'Absolute velocity';
    end
    if exist('type', 'var')
        if strcmp(type, 'streamline')
            h_stream = streamline(x, y, x_dot, y_dot, x(1:2:end, 1:2:end), y(1:2:end, 1:2:end));
        elseif strcmp(type, 'streamslice')
            h_stream = streamslice(x, y, x_dot, y_dot, 2, 'method', 'cubic');
        else
            error('Unsupported plot type');
        end
    else
        error("Set plot type ('streamline' or 'streamslice')");
    end
    set(h_stream, 'LineWidth', 1);
    set(h_stream, 'color', [0. 0. 0.]);
    set(h_stream, 'HandleVisibility', 'off');
    axis equal;
    axis tight
    scatter(x0(1, :), x0(2, :), 100, 'r*', 'LineWidth', 2);

    if exist('x_target', 'var')
        scatter(x_target(1, :), x_target(2, :), 100, 'bd', 'LineWidth', 2);
    end
end