clear all;
clc;
close all
cd '/home/robot/workspaces/planar_robot/data'
% src/mpc_pr/log_reward/pr_script_20210622_115923/';
data= load('data_22.csv');
q = data(:,1:2);
init = data(:,3:4);

% q_goal = data(:,3:4);
q_dot = data(:,6:7);
min_dist = data(:,8:15);
solutions = data(:,16:17);
% smallest_distance  = data(:,15);
% wall_dist = data(:,16:17);

len=length(q_dot);
for i=1:len
    q_goal(i,:)=[3.14, 0];
end
dt=0.1;
figure1 = figure('Name', 'positions')
subplot(2,1,1);
grid on;
hold on;
plot(q(:,1));
plot(q_goal(:,1));
set(gca,'XTick',0:100:100*len);
set(gca,'XTickLabel',0:dt*100:len*100*dt);

title("q 1")
subplot(2,1,2);
grid on;
hold on;
l1 = plot(q(:,2));
l2 = plot(q_goal(:,2));
set(gca,'XTick',0:100:100*len);
set(gca,'XTickLabel',0:dt*100:len*100*dt);

title("q 2")
% Construct a Legend with the data from the sub-plots
hL = legend([l1,l2],{"q", "q goal"});
% Programatically move the Legend
newPosition = [0.6 0.1 0.1 0.1];
newUnits = 'normalized';
set(hL,'Position', newPosition,'Units', newUnits);
saveas(figure1, 'joint_poses.png');

figure2 = figure('Name', 'velocities')
subplot(2,1,1);
grid on;
hold on;
plot(q_dot(:,1));
plot(solutions(:,1));
set(gca,'XTick',0:100:100*len);
set(gca,'XTickLabel',0:dt*100:len*100*dt);

title("q 1 dot")
subplot(2,1,2);
grid on;
hold on;
l1 = plot(q_dot(:,2));
l2 = plot(solutions(:,2));
set(gca,'XTick',0:100:100*len);
set(gca,'XTickLabel',0:dt*100:len*100*dt);

title("q 2 dot")
% Construct a Legend with the data from the sub-plots
hL = legend([l1,l2],{"q dot", "solutions"});
% Programatically move the Legend
newPosition = [0.6 0.1 0.1 0.1];
newUnits = 'normalized';
set(hL,'Position', newPosition,'Units', newUnits);
saveas(figure2, 'joint_vels.png');

% figure3 = figure('Name', 'minimum distances')
% subplot(4,2,1);
% grid on;
% hold on;
% plot(min_dist(:,1));
% set(gca,'XTick',0:100:100*len);
% set(gca,'XTickLabel',0:0.05*100:len*100*0.05);
% title("test point 1")
% subplot(4,2,2);
% grid on;
% hold on;
% plot(min_dist(:,2));
% set(gca,'XTick',0:100:100*len);
% set(gca,'XTickLabel',0:0.05*100:len*100*0.05);
% title("test point 2")
% subplot(4,2,3);
% grid on;
% hold on;
% plot(min_dist(:,3));
% set(gca,'XTick',0:100:100*len);
% set(gca,'XTickLabel',0:0.05*100:len*100*0.05);
% title("test point 3")
% subplot(4,2,4);
% grid on;
% hold on;
% plot(min_dist(:,4));
% set(gca,'XTick',0:100:100*len);
% set(gca,'XTickLabel',0:0.05*100:len*100*0.05);
% title("test point 4")
% saveas(figure3, 'minimum_distances.png');
% 
% figure3 = figure('Name', 'minimum distances')
% subplot(2,2,1);
% grid on;
% hold on;
% plot(wall_dist(:,1));
% set(gca,'XTick',0:100:100*len);
% set(gca,'XTickLabel',0:0.05*100:len*100*0.05);
% title("q1-wall dist")
% subplot(2,2,2);
% grid on;
% hold on;
% plot(wall_dist(:,2));
% set(gca,'XTick',0:100:100*len);
% set(gca,'XTickLabel',0:0.05*100:len*100*0.05);
% title("q2-wall dist")
% saveas(figure3, 'walldistances.png');
% 
% figure4 = figure('Name', 'smallest distance')
% hold on;
% plot(smallest_distance(:,1));
% set(gca,'XTick',0:100:100*len);
% set(gca,'XTickLabel',0:0.05*100:len*100*0.05);
% saveas(figure4, 'smallestdistance.png');