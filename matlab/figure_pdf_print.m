% ----- Init ----- %
f_title = 'Title';			% title of the figure
f_pos = [10 50 400 300]; 	% x, y, lx, ly
exp = 'experiment.mat';		% matlab variable or workspace

% ----- Figure creation ----- %
f = figure(f_title, f_pos);

% ----- Load data ----- %
load(exp);

% ----- Plot ----- %
% some plots.....
grid on 
title('plot_title')
xlabel('x_label') 
ylabel('y_label')
legend('leg1','leg2')

% ----- Print to pdf ----- %
set(f,'Units','Inches');
pos = get(f,'Position');
set(f,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(f,'-dpdf','-r0')
