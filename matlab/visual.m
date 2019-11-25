clear;
clc;
% close all;
v = load('VTTube.txt');
o = load('DTTube.txt');

Value = zeros(1638,3056);

for i = 1 : 1638
    for j = 1 : 3056
%         if(v(i,j) < 5  && o(i,j) < 5)
%             VT = 0;
%             DT = 0;
%         else
            VT = v(i,j);
            DT = o(i,j);
%         end
         Value(i,j) = 3000./(1+ exp( -(VT + DT - 9)/ 1.0 ) );
%          Value(i,j) = (max( VT, DT)*2)^0.1;
%             Value(i,j) =  30* VT + 50k00/(1+ DT);
    end
end


x = 1:1638;
y = 1:3056;



%%%%%%%%%%%%%%%%%%%% visualization %%%%%%%%%%%%%%%%%%%

figure;

%contour(y,x,VT(x,y));
contourf(y,x,Value(x,y),'LineColor', 'none');
%contour(y,x,DT(x,y));

colorbar;