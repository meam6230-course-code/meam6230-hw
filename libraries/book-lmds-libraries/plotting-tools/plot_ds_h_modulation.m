function plot_ds_h_modulation(x, y, x_dot, y_dot, h_x, x0, titleName, type, modulators)
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
    
    title(titleName,'Interpreter', 'LaTex', 'FontSize', 15)
%     h = pcolor(x,y,h_x);  
    nx = size(x,2); ny = size(y,2);
    h = surf(x,y,-0.0001*ones(nx,ny),h_x);
    set(h,'linestyle','none');
    load whiteCopperColorMap;
    colormap(cm);
    colorbar;

    c_bar = colorbar;
    c_bar.Label.String = 'Activation h(x)';

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
    axis tight;

    scatter(x0(1, :), x0(2, :), 100, 'r*', 'LineWidth', 2);

    if exist('modulators', 'var')
        scatter(modulators(1, :), modulators(2, :), 100, 'bd', 'LineWidth', 2);
    end
end