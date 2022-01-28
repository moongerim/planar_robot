clear all;
clc;
close all
cd '/home/robot/workspaces/planar_robot/_LOGS' 
%% Loss evalution
train_loss = load('train_log_20210723_101441.csv')
eval_loss = load('eval_log_20210723_101441.csv')
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
clear all
close all
clc
for k=1:1968
    filename = sprintf('data_%i.csv',k);
    data = load(filename);
    len(k,:) = length(data);
    init_pose(k,:) = data(2,1:2);
    goal_pose(k,:) = data(length(data),1:2);
    q = data(:,1:2);
%     init = data(:,3:4);
%     q_dot = data(:,6:7);
    min_dist = data(:,8:15);
    min_list(k,1) = min(min_dist(:,1));
    min_list(k,2) = min(min_dist(:,2));
    min_list(k,3) = min(min_dist(:,3));
    min_list(k,4) = min(min_dist(:,4));
    min_list(k,5) = min(min_dist(:,5));
    min_list(k,6) = min(min_dist(:,6));
    min_list(k,7) = min(min_dist(:,7));
    min_list(k,8) = min(min_dist(:,8));
    solutions = data(2:length(data),16:17);
    max_vel_1(k) = max(solutions(:,1));
    max_vel_2(k) = max(solutions(:,2));
    min_vel_1(k) = min(solutions(:,1));
    min_vel_2(k) = min(solutions(:,2));
%     len=length(q_dot);
%     dt=0.1;
%     figure1 = figure('Name', 'velo');
%     subplot(2,1,1);
%     grid on;
%     hold on;
%     plot(solutions(:,1));
%     set(gca,'XTick',0:100:100*len);
%     set(gca,'XTickLabel',0:dt*100:len*100*dt);
% 
%     title("q dot 1");
%     subplot(2,1,2);
%     grid on;
%     hold on;
%     l1 = plot(solutions(:,2));
%     set(gca,'XTick',0:100:100*len);
%     set(gca,'XTickLabel',0:dt*100:len*100*dt);
% 
%     title("q dot 2")
%     figurename = sprintf('jv_%i.png',k);
%     saveas(figure1, figurename);
end
figure
hold on;
plot(init_pose(:,1), init_pose(:,2),'linestyle','none','marker','o')
[a,b] = max(len)
[c,d] = min(len)

figure
hold on;
plot(goal_pose(:,1), goal_pose(:,2),'linestyle','none','marker','*')
iter = 1;
for i=1:1968
    if max_vel_1(i)>2.1 || max_vel_2(i)>2 || min_vel_1(i)<-2 || min_vel_2(i)<-2.1
        LIST_er(iter)= i
        iter=iter+1;
    end
end

for i=1:40
    k = LIST_er(i)
    filename = sprintf('data_%i.csv',k);
    data = load(filename);
    len = length(data);
    solutions = data(2:len,16:17);
    q_dot = data(2:len,6:7);
    dt=0.1;
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
    filename = sprintf('jv_%i.png',k);
    saveas(figure2, filename);
end
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