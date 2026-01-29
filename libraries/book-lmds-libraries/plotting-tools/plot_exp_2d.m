function [h_exp] = plot_exp_2d(fig, exp_handle)

nx = 400; ny = 400;

handle(fig);
axlim = [get(gca,'Xlim'), get(gca, 'Ylim')];
ax_x=linspace(axlim(1),axlim(2),nx); %computing the mesh points along each axis
ax_y=linspace(axlim(3),axlim(4),ny); %computing the mesh points along each axis
[x_tmp, y_tmp]=meshgrid(ax_x,ax_y);  %meshing the input domain
x=[x_tmp(:), y_tmp(:)]';

% Old implementation assuming exp_handle can handle batch evaluation
% [h_x] = feval(exp_handle,x);

% New way for MEAM 6230 Spring 2025 HW
h_x = arrayfun(@(k) exp_handle(x(:,k)) , 1:size(x, 2));
h_exp = reshape(h_x,nx,ny);

% Cap the plotting function
% h_exp(h_exp < 0) = 0;

hold on;
% h = pcolor(x_tmp,y_tmp,h_exp);
h = surf(x_tmp,y_tmp,-0.0001*ones(nx,ny),h_exp);
set(h,'linestyle','none');
load whiteCopperColorMap;
colormap(cm);
colorbar;

min_h = min(min(h_exp));
max_h = max(max(h_exp));

if min_h == max_h
    caxis([min_h, max_h+1]);
else
    caxis([min_h, max_h]);
end


end