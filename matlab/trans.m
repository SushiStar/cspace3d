% distance transform

pic=imread('triangle.png');
tmap = rgb2gray(pic);


for  i = 1:numel(tmap)
    if(tmap(i) <= 200)
        tmap(i) = 1;
    else
        tmap(i) = 0;
    end
    
end

[a,b] = size(tmap);

for i = 1:b
    tmap(1,i) = 1;
    tmap(a,i) = 1;
end

for i = 1:a
    tmap(i, 1) = 1;
    tmap(i, b) = 1;
end


% transform value
D1 = bwdist(tmap,'euclidean');
D = round(D1);

fileID1 = fopen('DTTriangle.txt','w');


for i = 1:a
    
    m = D(i,:);
    %fprintf(fileID, '%.2f ', m );
    fprintf(fileID1, '%d ', m );
    fprintf(fileID1, '\n');
end

fclose(fileID1);

m = 3;
V = zeros(a,b);
% voronoi transform
for i  = 1+ m : a-m
   for j = 1+m : b-m
       if( ( D1(i,j) > D1(i, j-m) && D1(i,j) > D1(i, j+m) ) || ( D1(i,j) > D1(i-m, j) && D1(i,j) > D1(i+m, j) ) || ( D1(i,j) > D1(i-m, j-m) && D1(i,j) > D1(i+m, j+m) ) || ( D1(i,j) > D1(i+m, j-m) && D1(i,j) > D1(i-m, j+m) )   ) 
           V(i,j) = 1;
       end
        
   end
end

D2 =bwdist(V, 'euclidean');
D2 = round(D2);
fileID2 = fopen('VTTriangle.txt','w');

for i = 1:a
    m = D2(i,:);
    fprintf(fileID2, '%d ', m);
    fprintf(fileID2, '\n');
end
fclose(fileID2);
