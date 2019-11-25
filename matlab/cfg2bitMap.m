function out = cfg2bitMap(file_dir)

fid = fopen(file_dir);
line = fgetl(fid);
dim = regexp(line,'\d*','Match');
height = str2num(dim{1});
width = str2num(dim{2});

out = zeros(width,height);

for i = 1:9
    line = fgetl(fid);
end

for counter = 1 : width
    line = fgetl(fid);
    out(counter,:) = str2num(line);
end

