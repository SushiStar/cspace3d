%{
 {Results of 7 maps plotting the planning time as a function of 
 {Penalty radius.
 { long motion primitive is used.
 %}

x = 0:0.2:18;
circle = load('time_circle.txt');
circlex = x(1:size(circle));
mesh = load('time_mesh.txt');
meshx = x(1:size(mesh));
maze =  load('time_maze.txt');           
mazex = x(1:size(maze));
square = load('time_square.txt');          
squarex = x(1:size(square));
spiral = load('time_spiral.txt');          
spiralx = x(1:size(spiral));
triangle = load('time_triangle.txt');        
trianglex = x(1:size(triangle));
tube = load('time_tube.txt');            
tubex = x(1:size(tube));
                   
                   
figure;            
plot(circlex, circle);
hold on;           
plot(meshx, mesh);
plot(mazex, maze);
plot(squarex, square);
plot(spiralx, spiral);
plot(trianglex, triangle);
plot(tubex, tube);

legend('circle', 'mesh', 'maze', 'square', 'spiral', 'triangle', 'tube');
xlabel('Radius(times of step size)');
ylabel('Planning time(s)');
title('Nearest Neighbor Checking Radius vs Planning Time');
