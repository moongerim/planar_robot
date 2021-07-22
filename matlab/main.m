clear all;
clc;
close all
cd '/home/robot/workspaces/planar_robot/_LOGS' 
%% Loss evalution
train_loss = load('train_log_20210701_105701.csv')
eval_loss = load('eval_log_20210701_105701.csv')
figure_0 = figure('Name', 'losses')
hold on
plot(train_loss)
plot(eval_loss)
legend('train loss', 'eval loss')

%% Test sample
test = load('test_log_pytorch_train_20210630_161504.csv')
predicted_q_dot = test(:,1:2)
real_q_dot = test(:,3:4)
q = test(:,5:6)

len = length(q)
dt = 0.1
figure2 = figure('Name', 'velocities')
subplot(2,1,1);
grid on;
hold on;
plot(predicted_q_dot(:,1));
plot(real_q_dot(:,1));
set(gca,'XTick',0:100:100*len);
set(gca,'XTickLabel',0:dt*100:len*100*dt);
title("q 1 dot")

subplot(2,1,2);
grid on;
hold on;
l1 = plot(predicted_q_dot(:,2));
l2 = plot(real_q_dot(:,2));
set(gca,'XTick',0:100:100*len);
set(gca,'XTickLabel',0:dt*100:len*100*dt);
title("q 2 dot")

% Construct a Legend with the data from the sub-plots
hL = legend([l1,l2],{"predicted q dot", "real q dot"});
% Programatically move the Legend
newPosition = [0.6 0.1 0.1 0.1];
newUnits = 'normalized';
set(hL,'Position', newPosition,'Units', newUnits);

figure1 = figure('Name', 'positions');
subplot(2,1,1);
grid on;
hold on;
plot(q(:,1));
set(gca,'XTick',0:100:100*len);
set(gca,'XTickLabel',0:dt*100:len*100*dt);

title("q 1");
subplot(2,1,2);
grid on;
hold on;
l1 = plot(q(:,2));
set(gca,'XTick',0:100:100*len);
set(gca,'XTickLabel',0:dt*100:len*100*dt);

title("q 2")

%% Data preprocessing
for k=1:855
    filename = sprintf('data_%i.csv',k);
    data = load(filename);
    len(k,:) = length(data);
    init_pose(k,:) = data(2,1:2);
%     q = data(:,1:2);
%     init = data(:,3:4);
%     q_dot = data(:,6:7);
%     min_dist = data(:,8:15);
%     solutions = data(:,16:17);
%     max_vel_1(k) = max(solutions(:,1))
%     max_vel_2(k) = max(solutions(:,2))
%     len=length(q_dot);
%     dt=0.1;
%     figure1 = figure('Name', 'positions');
%     subplot(2,1,1);
%     grid on;
%     hold on;
%     plot(q(:,1));
%     set(gca,'XTick',0:100:100*len);
%     set(gca,'XTickLabel',0:dt*100:len*100*dt);
% 
%     title("q 1");
%     subplot(2,1,2);
%     grid on;
%     hold on;
%     l1 = plot(q(:,2));
%     set(gca,'XTick',0:100:100*len);
%     set(gca,'XTickLabel',0:dt*100:len*100*dt);
% 
%     title("q 2")
%     figurename = sprintf('jp_%i.png',k);
%     saveas(figure1, figurename);
end
figure
hold on;
plot(init_pose(:,1), init_pose(:,2),'linestyle','none','marker','o')
[a,b] = max(len)
[c,d] = min(len)
% figure2 = figure('Name', 'velocities')
% subplot(2,1,1);
% grid on;
% hold on;
% plot(q_dot(:,1));
% plot(solutions(:,1));
% set(gca,'XTick',0:100:100*len);
% set(gca,'XTickLabel',0:dt*100:len*100*dt);
% 
% title("q 1 dot")
% subplot(2,1,2);
% grid on;
% hold on;
% l1 = plot(q_dot(:,2));
% l2 = plot(solutions(:,2));
% set(gca,'XTick',0:100:100*len);
% set(gca,'XTickLabel',0:dt*100:len*100*dt);
% 
% title("q 2 dot")
% % Construct a Legend with the data from the sub-plots
% hL = legend([l1,l2],{"q dot", "solutions"});
% % Programatically move the Legend
% newPosition = [0.6 0.1 0.1 0.1];
% newUnits = 'normalized';
% set(hL,'Position', newPosition,'Units', newUnits);
% saveas(figure2, 'joint_vels.png');

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