clear;
clc;

linewidth = 2;
fontName = 'times new roman';
fontSize = 15;

barl = load('barL.txt');
bars = load('barS.txt');
dijl = load('DijL.txt');
dijs = load('DijS.txt');
pabl = load('pabL.txt');
pabs = load('pabS.txt');
penl = load('penL.txt');
pens = load('penS.txt');

rrtl = load('rrtl/log1.txt')';
rrtl = [rrtl; load('rrtl/log2.txt')'];
rrtl = [rrtl; load('rrtl/log3.txt')'];
rrtl = [rrtl; load('rrtl/log4.txt')'];
rrtl = [rrtl; load('rrtl/log5.txt')'];

rrts = load('rrts/log1.txt')';
rrts = [rrts; load('rrts/log2.txt')'];
rrts = [rrts; load('rrts/log3.txt')'];
rrts = [rrts; load('rrts/log4.txt')'];
rrts = [rrts; load('rrts/log5.txt')'];


rrtltime = [rrtl(3,:); rrtl(6,:); rrtl(9,:); rrtl(12,:); rrtl(15,:)];
rrtlsample = [rrtl(1,:); rrtl(4,:); rrtl(7,:); rrtl(10,:); rrtl(13,:)]; 
rrtlcost = [rrtl(2,:); rrtl(5,:); rrtl(8,:); rrtl(11,:); rrtl(14,:)];

[a aa] = min(rrtltime,[], 1);
b = [];
c = [];
for i = 1:length(aa)
    b = [b; rrtlcost(aa(i) ,i )]; 
    c = [c; rrtlsample(aa(i), i)];
end
rrtl = [c b a'];


rrtstime = [rrts(3,:); rrts(6,:); rrts(9,:); rrts(12,:); rrts(15,:)];
rrtssample = [rrts(1,:); rrts(4,:); rrts(7,:); rrts(10,:); rrts(13,:)]; 
rrtscost = [rrts(2,:); rrts(5,:); rrts(8,:); rrts(11,:); rrts(14,:)];

[a aa] = min(rrtstime,[], 1);
b = [];
c = [];
for i = 1:length(aa)
    b = [b; rrtscost(aa(i) ,i )]; 
    c = [c; rrtssample(aa(i), i)];
end
rrts = [c b a'];


%---------------------------- sort -----------------------------
% exp cost time
[temp, order1] = sort(penl(:,3) + pabl(:,3));
barl = barl(order1, :);
dijl = dijl(order1, :);
pabl = pabl(order1, :);
penl = penl(order1, :);
rrtl = rrtl(order1, :);

barl(find(barl == 0)) = 120;

[temp, order2] = sort(pens(:,3) + pabs(:,3));
bars = bars(order2, :);
dijs = dijs(order2, :);
pabs = pabs(order2, :);
pens = pens(order2, :);
rrts = rrts(order2, :);

bars(find(bars == 0)) = 120;

%--------------------------- partition ---------------------------
% 25 + 25 + 20/23
time_easyL =   [barl( 1:25,3), dijl( 1:25,3), pabl( 1:25,3), penl( 1:25,3), rrtl( 1:25,3)];
time_mediumL = [barl(26:50,3), dijl(26:50,3), pabl(26:50,3), penl(26:50,3), rrtl(26:50,3)];
time_hardL =   [barl(51:70,3), dijl(51:70,3), pabl(51:70,3), penl(51:70,3), rrtl(51:70,3)];
cost_easyL =   [barl( 1:25,2), dijl( 1:25,2), pabl( 1:25,2), penl( 1:25,2), rrtl( 1:25,2)];
cost_mediumL = [barl(26:50,2), dijl(26:50,2), pabl(26:50,2), penl(26:50,2), rrtl(26:50,2)];
cost_hardL =   [barl(51:70,2), dijl(51:70,2), pabl(51:70,2), penl(51:70,2), rrtl(51:70,2)];
exps_easyL =   [barl( 1:25,1), dijl( 1:25,1), pabl( 1:25,1), penl( 1:25,1), rrtl( 1:25,1)];
exps_mediumL = [barl(26:50,1), dijl(26:50,1), pabl(26:50,1), penl(26:50,1), rrtl(26:50,1)];
exps_hardL =   [barl(51:70,1), dijl(51:70,1), pabl(51:70,1), penl(51:70,1), rrtl(51:70,1)];
succ_easyL =   1-[sum(cost_easyL(:,1) >= 10000000), sum(cost_easyL(:,2) >= 10000000),  ...
                sum(cost_easyL(:,3) >= 10000000), sum(cost_easyL(:,4) >= 10000000),  ...
                sum(cost_easyL(:,5) >= 10000000)]./25;
succ_mediumL = 1-[sum(cost_mediumL(:,1) >= 10000000), sum(cost_mediumL(:,2) >= 10000000),  ...
                sum(cost_mediumL(:,3) >= 10000000), sum(cost_mediumL(:,4) >= 10000000),  ...
                sum(cost_mediumL(:,5) >= 10000000)]./25;
succ_hardL =   1-[sum(cost_hardL(:,1) >= 10000000), sum(cost_hardL(:,2) >= 10000000),  ...
                sum(cost_hardL(:,3) >= 10000000), sum(cost_hardL(:,4) >= 10000000),  ...
                sum(cost_hardL(:,5) >= 10000000)]./20;


time_easyS =   [bars( 1:25,3), dijs( 1:25,3), pabs( 1:25,3), pens( 1:25,3), rrts( 1:25,3)];
time_mediumS = [bars(26:50,3), dijs(26:50,3), pabs(26:50,3), pens(26:50,3), rrts(26:50,3)];
time_hardS =   [bars(51:73,3), dijs(51:73,3), pabs(51:73,3), pens(51:73,3), rrts(51:73,3)];
cost_easyS =   [bars( 1:25,2), dijs( 1:25,2), pabs( 1:25,2), pens( 1:25,2), rrts( 1:25,2)];
cost_mediumS = [bars(26:50,2), dijs(26:50,2), pabs(26:50,2), pens(26:50,2), rrts(26:50,2)];
cost_hardS =   [bars(51:73,2), dijs(51:73,2), pabs(51:73,2), pens(51:73,2), rrts(51:73,2)];
exps_easyS =   [bars( 1:25,1), dijs( 1:25,1), pabs( 1:25,1), pens( 1:25,1), rrts( 1:25,1)];
exps_mediumS = [bars(26:50,1), dijs(26:50,1), pabs(26:50,1), pens(26:50,1), rrts(26:50,1)];
exps_hardS =   [bars(51:73,1), dijs(51:73,1), pabs(51:73,1), pens(51:73,1), rrts(51:73,1)];
succ_easyS =   1-[sum(cost_easyS(:,1) >= 10000000), sum(cost_easyS(:,2) >= 10000000),  ...
                sum(cost_easyS(:,3) >= 10000000), sum(cost_easyS(:,4) >= 10000000),  ...
                sum(cost_easyS(:,5) >= 10000000)]./25;
succ_mediumS = 1-[sum(cost_mediumS(:,1) >= 10000000), sum(cost_mediumS(:,2) >= 10000000),  ...
                sum(cost_mediumS(:,3) >= 10000000), sum(cost_mediumS(:,4) >= 10000000),  ...
                sum(cost_mediumS(:,5) >= 10000000)]./25;
succ_hardS =   1-[sum(cost_hardS(:,1) >= 10000000), sum(cost_hardS(:,2) >= 10000000),  ...
                sum(cost_hardS(:,3) >= 10000000), sum(cost_hardS(:,4) >= 10000000),  ...
                sum(cost_hardS(:,5) >= 10000000)]./23;


%--------------------------- plot ---------------------------

%*** Large
%G1 = [ones(1,25)';  2*ones(1,25)'; 3*ones(1,25)'; 4*ones(1,25)'; 5*ones(1,25)'];
%G2 = [ones(1,20)';  2*ones(1,20)'; 3*ones(1,20)'; 4*ones(1,20)'; 5*ones(1,20)'];


%h=figure(1); hold on;
%set(gcf,'Position',[100 100 250 350]);
%set(gca, 'fontname', fontName, 'fontsize', fontSize);
%h1 = boxplot(time_easyL, G1, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
%grid on;
%ylabel('Planning Time(s)')
%set(gca,'XTickLabelRotation',45)
%ylim([0 0.5]);
%set(h,'PaperSize',[3.5 5]); %set the paper size to what you want  
%print(h,'~/Desktop/time_easyL.pdf','-dpdf') % then print it


%h=figure(2); hold on;
%set(gcf,'Position',[100 100 250 350]);
%set(gca, 'fontname', fontName, 'fontsize', fontSize);
%h2 = boxplot(time_mediumL, G1, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
%grid on;
%ylabel('Planning Time(s)')
%set(gca,'XTickLabelRotation',45)
%ylim([0 2]);
%set(h,'PaperSize',[3.5 5]); %set the paper size to what you want  
%print(h,'~/Desktop/time_mediumL.pdf','-dpdf') % then print it


%h=figure(3); hold on;
%set(gcf,'Position',[100 100 250 350]);
%set(gca, 'fontname', fontName, 'fontsize', fontSize);
%h3 = boxplot(time_hardL, G2, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
%grid on;
%ylabel('Planning Time(s)')
%set(gca,'XTickLabelRotation',45)
%ylim([0 10]);
%set(h,'PaperSize',[3.5 5]); %set the paper size to what you want  
%print(h,'~/Desktop/time_hardL.pdf','-dpdf') % then print it

%h=figure(4); hold on;
%set(gcf,'Position',[100 100 250 350]);
%set(gca, 'fontname', fontName, 'fontsize', fontSize);
%h1 = boxplot(cost_easyL, G1, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
%grid on;
%ylabel('Solution Cost')
%set(gca,'XTickLabelRotation',45)
%ylim([0 15e4]);
%set(h,'PaperSize',[3.5 5]); %set the paper size to what you want  
%print(h,'~/Desktop/cost_easyL.pdf','-dpdf') % then print it

%h=figure(5); hold on;
%set(gcf,'Position',[100 100 250 350]);
%set(gca, 'fontname', fontName, 'fontsize', fontSize);
%h1 = boxplot(cost_mediumL, G1, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
%grid on;
%ylabel('Solution Cost')
%set(gca,'XTickLabelRotation',45)
%ylim([0 15e4]);
%set(h,'PaperSize',[3.5 5]); %set the paper size to what you want  
%print(h,'~/Desktop/cost_mediumL.pdf','-dpdf') % then print it

%h=figure(6); hold on;
%set(gcf,'Position',[100 100 250 350]);
%set(gca, 'fontname', fontName, 'fontsize', fontSize);
%h1 = boxplot(cost_hardL, G2, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
%grid on;
%ylabel('Solution Cost')
%set(gca,'XTickLabelRotation',45)
%ylim([0 15e4]);
%set(h,'PaperSize',[3.5 5]); %set the paper size to what you want  
%print(h,'~/Desktop/cost_hardL.pdf','-dpdf') % then print it

%{
 {c = {'', 'Barraquand', 'BFS3D', 'Pablo', 'Penalty', 'RRT',''};
 {figure(7); hold on;
 {set(gcf,'Position',[100 100 250 350]);
 {set(gca, 'fontname', fontName, 'fontsize', fontSize);
 {bar(succ_easyL);
 {set(gca, 'xticklabel', c);
 {ylabel('Success rate')
 {set(gca,'XTickLabelRotation',60)
 {grid on;
 {
 {figure(8); hold on;
 {set(gcf,'Position',[100 100 250 350]);
 {set(gca, 'fontname', fontName, 'fontsize', fontSize);
 {bar(succ_mediumL);
 {set(gca, 'xticklabel', c);
 {ylabel('Success rate')
 {set(gca,'XTickLabelRotation',60)
 {grid on;
 {
 {figure(9); hold on;
 {set(gcf,'Position',[100 100 250 350]);
 {set(gca, 'fontname', fontName, 'fontsize', fontSize);
 {bar(succ_hardL);
 {set(gca, 'xticklabel', c);
 {ylabel('Success rate')
 {set(gca,'XTickLabelRotation',60)
 {grid on;
 {
 {
 %}

%figure(7); hold on;
%h1 = boxplot(exps_easyL, G1, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
%grid on;
%ylabel('Number of expansions')
%set(gca,'XTickLabelRotation',45)

%figure(8); hold on;
%h1 = boxplot(exps_mediumL, G1, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
%grid on;
%ylabel('Number of expansions')
%set(gca,'XTickLabelRotation',45)

%figure(9); hold on;
%h1 = boxplot(exps_hardL, G2, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
%grid on;
%ylabel('Number of expansions')
%set(gca,'XTickLabelRotation',45)

%*** Small
G1 = [ones(1,25)';  2*ones(1,25)'; 3*ones(1,25)'; 4*ones(1,25)'; 5*ones(1,25)'];
G2 = [ones(1,23)';  2*ones(1,23)'; 3*ones(1,23)'; 4*ones(1,23)'; 5*ones(1,23)'];

%ylim([0 0.5]);
%set(h,'PaperSize',[3.5 5]); %set the paper size to what you want  
%print(h,'~/Desktop/time_easyL.pdf','-dpdf') % then print it



h=figure(1); hold on;
set(gcf,'Position',[100 100 250 350]);
set(gca, 'fontname', fontName, 'fontsize', fontSize);
h1 = boxplot(time_easyS, G1, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
grid on;
ylabel('Planning Time(s)')
set(gca,'XTickLabelRotation',45)
ylim([0 0.5]);
set(h,'PaperSize',[3.5 5]); %set the paper size to what you want  
print(h,'~/Desktop/time_easyS.pdf','-dpdf') % then print it


h=figure(2); hold on;
set(gcf,'Position',[100 100 250 350]);
set(gca, 'fontname', fontName, 'fontsize', fontSize);
h2 = boxplot(time_mediumS, G1, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
grid on;
ylabel('Planning Time(s)')
set(gca,'XTickLabelRotation',45)
ylim([0 2]);
set(h,'PaperSize',[3.5 5]); %set the paper size to what you want  
print(h,'~/Desktop/time_mediumS.pdf','-dpdf') % then print it


h=figure(3); hold on;
set(gcf,'Position',[100 100 250 350]);
set(gca, 'fontname', fontName, 'fontsize', fontSize);
h3 = boxplot(time_hardS, G2, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
grid on;
ylabel('Planning Time(s)')
set(gca,'XTickLabelRotation',45)
ylim([0 2]);
set(h,'PaperSize',[3.5 5]); %set the paper size to what you want  
print(h,'~/Desktop/time_hardS.pdf','-dpdf') % then print it



h=figure(4); hold on;
set(gcf,'Position',[100 100 250 350]);
set(gca, 'fontname', fontName, 'fontsize', fontSize);
h1 = boxplot(cost_easyS, G1, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
grid on;
ylabel('Solution Cost')
set(gca,'XTickLabelRotation',45)
ylim([0 15e4]);
set(h,'PaperSize',[3.5 5]); %set the paper size to what you want  
print(h,'~/Desktop/cost_easy_S.pdf','-dpdf') % then print it


h=figure(5); hold on;
set(gcf,'Position',[100 100 250 350]);
set(gca, 'fontname', fontName, 'fontsize', fontSize);
h1 = boxplot(cost_mediumS, G1, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
grid on;
ylabel('Solution Cost')
set(gca,'XTickLabelRotation',45)
ylim([0 15e4]);
set(h,'PaperSize',[3.5 5]); %set the paper size to what you want  
print(h,'~/Desktop/cost_medium_S.pdf','-dpdf') % then print it


h=figure(6); hold on;
set(gcf,'Position',[100 100 250 350]);
set(gca, 'fontname', fontName, 'fontsize', fontSize);
h1 = boxplot(cost_hardS, G2, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
grid on;
ylabel('Solution Cost')
set(gca,'XTickLabelRotation',45)
ylim([0 15e4]);
set(h,'PaperSize',[3.5 5]); %set the paper size to what you want  
print(h,'~/Desktop/cost_hardS.pdf','-dpdf') % then print it


%{
 {c = {'Barraquand', 'BFS3D', 'Pablo', 'Penalty', 'RRT'};
 {figure(7); hold on;
 {set(gcf,'Position',[100 100 250 350]);
 {set(gca, 'fontname', fontName, 'fontsize', fontSize);
 {bar(succ_easyS);
 {set(gca, 'xticklabel', c);
 {ylabel('Success rate')
 {set(gca,'XTickLabelRotation',45)
 {grid on;
 {
 {c = {'Barraquand', 'BFS3D', 'Pablo', 'Penalty', 'RRT'};
 {figure(8); hold on;
 {set(gcf,'Position',[100 100 250 350]);
 {set(gca, 'fontname', fontName, 'fontsize', fontSize);
 {bar(succ_mediumS);
 {set(gca, 'xticklabel', c);
 {ylabel('Success rate')
 {set(gca,'XTickLabelRotation',45)
 {grid on;
 {
 {c = {'Barraquand', 'BFS3D', 'Pablo', 'Penalty', 'RRT'};
 {figure(9); hold on;
 {set(gcf,'Position',[100 100 250 350]);
 {set(gca, 'fontname', fontName, 'fontsize', fontSize);
 {bar(succ_hardS);
 {set(gca, 'xticklabel', c);
 {ylabel('Success rate')
 {set(gca,'XTickLabelRotation',45)
 {grid on;
 {
 %}


%{
 {figure(7); hold on;
 {set(gcf,'Position',[100 100 250 350]);
 {set(gca, 'fontname', fontName, 'fontsize', fontSize);
 {h1 = boxplot(exps_easyS, G1, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
 {grid on;
 {ylabel('Number of expansions')
 {set(gca,'XTickLabelRotation',45)
 {
 {figure(8); hold on;
 {set(gcf,'Position',[100 100 250 350]);
 {set(gca, 'fontname', fontName, 'fontsize', fontSize);
 {h1 = boxplot(exps_mediumS, G1, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
 {grid on;
 {ylabel('Number of expansions')
 {set(gca,'XTickLabelRotation',45)
 {
 {figure(9); hold on;
 {set(gcf,'Position',[100 100 250 350]);
 {set(gca, 'fontname', fontName, 'fontsize', fontSize);
 {h1 = boxplot(exps_hardS, G2, 'Labels', {'Barraquand',  'BFS3D', 'Pablo', 'Penalty','RRT'}, 'Symbol','');
 {grid on;
 {ylabel('Number of expansions')
 {set(gca,'XTickLabelRotation',45)
 %}

