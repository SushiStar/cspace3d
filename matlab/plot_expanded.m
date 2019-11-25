function rlt = plot_expanded(mapdir, statesdir);
img = cfg2bitMap(mapdir);
img = 1-img;
imshow(img);
hold on;

debug_state = load(statesdir);
x = debug_state(:,1);
x = x./0.025;
y = debug_state(:,2);
y = y./0.025;  
z = debug_state(:,3);

% plot mode: 1 plot with direction /  2 plot heatmap
plotmode = 1;

if 1 == plotmode        % plot arrows
    dx = cos(z);
    dy = sin(z);
   
    a = 0;

    if(0 == a )   % plot at once
        %quiver(x, y, dx, dy, 0.1,'r');
        plot(x, y, '.r');
    else 
        pause on;
        num = size(x);
         for i = 1:num
            quiver(x(i), y(i), dx(i), dy(i) );
               plot(x(i),y(i), 'o');
            pause(0.000001);
            hold on;
        end
    end

else                    % plot density
    heatscatter(x,y);
end

set(gca,'Ydir','normal')

rlt = 0;
