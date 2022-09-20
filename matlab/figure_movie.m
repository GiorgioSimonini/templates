%% setup
% movie parameters
movie_mode = 0;     % 1 creates the video file
movie_title = 'file_title';
movie_fps = 30;		% movie frame per second

%% Animation
fps = movie_fps;
period=1/fps;
k = 1;				% init of movie's frames counter

tprec=0;
index=1;
t = out.times;		% from simulink
sizet=size(t);

%% Graphics
f = figure('name', 'MovieFigure');

% plot config
grid on;
xlabel('x');
ylabel('y');
zlabel('z');
%hold on

pause();

while index < sizet(1)
    %cla
    while t(index) < (tprec+period) 
        if index >= sizet(1)
            break 
        else 
            index = index + 1;
        end
    end
	
    % ----- stuff ----- %
	% plot something
	% ----- ----- ----- %

    tprec=t(index);
    drawnow;
    pause(period);
    index = index +1;
    
    % animation
    if movie_mode == 1
		movieVector(k) = getframe(figh);
    elseif movie_mode == 2
        movieVector(k) = getframe(figh, [10,10,1910,960]);
    end		
	k = k+1;
    
end

% movie generator
if movie_mode == 1
    if movie_mode
        movie = VideoWriter( movie_title, 'MPEG-4');
        movie.FrameRate = movie_fps;

        open(movie);
        writeVideo(movie, movieVector);
        close(movie);
        disp([movie_title ' saved!'])
    end
end