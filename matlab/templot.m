%%%%%% to plot the expansions of the results
%%%%%% not to used to visual the results

clear;
clc;
h = figure;
%debug_state = load('~/cspace/results_states/Dijk/Dijk_L/results/tube/tube_exp.txt');
debug_state = load('~/research/DuplicateDetection/cspace/build/debug_state.txt');
%debug_state = load('~/cspace/results_states/RRT/RRT_S/results/tube/tube_block_exp1.txt');
%solution = load('~/research/DuplicateDetection/cspace/build/sol.txt');

%img = imread('../map/tube/tube_block_G.png');
img = imread('../map/hand_made_maps/tube/tube_G.png');

img(img > 200) = 255;
img(img <= 200) = 100;
[width hight] = size(img);
    

image(img);


hold on;

x = debug_state(:,1);
x = x./0.025;
y = debug_state(:,2);
y = y./0.025;  
z = debug_state(:,3);

% plot mode: 1 plot with direction /  2 plot heatmap
plotmode = 2;

if 1 == plotmode        % plot arrows
    dx = cos(z);
    dy = sin(z);
   
    a = 0;

    if(0 == a )   % plot at once
        quiver(x, y, dx, dy, 0.1);
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
    caxis([0 140])
end
%plot(solution(:,1)./0.025, solution(:,2)./0.025, 'r.', 'MarkerSize', 1);
set(h, 'Position', [300 300 600 300]);
set(gca,'Ydir','normal')
%axis([0 3100 -10 1650]);
set(gca, 'XTick',[]);
set(gca, 'YTick',[]);
set(h,'PaperSize',[7.5 4]); %set the paper size to what you want  
print(h,'~/Desktop/WA3.pdf','-dpdf') % then print it


