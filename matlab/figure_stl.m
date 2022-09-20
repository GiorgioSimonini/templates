addpath('utils')
%% load stl
obj = patch(stlread('xxx_stl_path_xxx'),...
    'EdgeColor','none', 'FaceLighting','gouraud');

%% Graphics
figh = figure('name', 'Figure_stl');
% set(	gca, 'drawmode', 'fast');
% lighting phong;
% set(gcf, 'Renderer', 'zbuffer');

camlight('headlight');
material('dull');

%% transformation matrices
A0 = [  1 0 0 0; ...
        0 1 0 0; ...
        0 0 1 0; ...
        0 0 0 1];
A1 = [xxx];

% transform 0
obj_T = hgtransform('Parent',gca);
set(obj_T,'Matrix', A0);
set(obj,'Parent', obj_T);
set(obj, 'facec', [255,255,255]./255);

% plot config
View = [30 20];				% initial view
axis equal;
grid on;
xlabel('x');
ylabel('y');
zlabel('z');
%hold on
view(View(1, 1), View(1, 2));

% transform 1
set(obj_T,'Matrix', A1);
set(obj,'Parent', obj_T);
