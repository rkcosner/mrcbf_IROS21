%% retrieve stuff
clear all; close all
set(0, "DefaultLineLinewidth", 2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n = 4;
m = 1;
alpha_bs = 5; 
alpha = .25; 
t_max = 25; 

bag = rosbag('check_tamas.bag');

bSel = select(bag,'Topic','/segway_sim/state');
msg_structs = readMessages(bSel, "DataFormat", "struct");
for i = 1:length(msg_structs)
    t_x_cbf(i) = double(msg_structs{i}.Time)/1000000;
    x_cbf(:,i) = msg_structs{i}.StateVec;
    if t_x_cbf(i) >t_max
        break
    end
end
x_cbf = x_cbf';

bSel = select(bag,'Topic','/segway_sim/input');
msg_structs = readMessages(bSel, "DataFormat", "struct");
t0 = double(msg_structs{1}.Header.Stamp.Sec) + 10^-9*double(msg_structs{1}.Header.Stamp.Nsec);
for i = 1:length(msg_structs)
    t_u_cbf(i) = double(msg_structs{i}.Header.Stamp.Sec) + 10^-9*double(msg_structs{i}.Header.Stamp.Nsec) - double(t0);
    u_cbf(:,i) = msg_structs{i}.InputVec;
    if t_u_cbf(i)>t_max
        break
    end
end


bag = rosbag('mrcbf_test2.bag');

bSel = select(bag,'Topic','/segway_sim/state');
msg_structs = readMessages(bSel, "DataFormat", "struct");
for i = 1:length(msg_structs)
    t_x_mrcbf(i) = double(msg_structs{i}.Time)/1000000;
    x_mrcbf(:,i) = msg_structs{i}.StateVec;
    if t_x_mrcbf(i)>t_max
        break
    end
end
x_mrcbf = x_mrcbf';

bSel = select(bag,'Topic','/segway_sim/input');
msg_structs = readMessages(bSel, "DataFormat", "struct");
t0 = double(msg_structs{1}.Header.Stamp.Sec) + 10^-9*double(msg_structs{1}.Header.Stamp.Nsec);
for i = 1:length(msg_structs)
    t_u_mrcbf(i) = double(msg_structs{i}.Header.Stamp.Sec) + 10^-9*double(msg_structs{i}.Header.Stamp.Nsec) - double(t0);
    u_mrcbf(:,i) = msg_structs{i}.InputVec;
    if t_u_mrcbf(i) > t_max
        break
    end
end



%%


figure
hold on 
safe_set = patch([0,0,t_max,t_max], [-2, 1, 1, -2], 'g', 'EdgeColor', 'none')
robust_region = patch([t_x_mrcbf, flip(t_x_mrcbf)] , [x_mrcbf(:,1);flip(x_mrcbf(:,1))-0.4], 'r', 'EdgeColor', 'none')
safe_set.FaceAlpha = 0.3; 
robust_region.FaceAlpha = 0.3; 

cbf = plot(t_x_cbf, x_cbf(:,1), 'b')
cbf_hat = plot(t_x_cbf, x_cbf(:,1) - 0.2, 'b--')
mrcbf = plot(t_x_mrcbf, x_mrcbf(:,1), 'r')
mrcbf_hat = plot(t_x_mrcbf, x_mrcbf(:,1)-0.2, 'r--')
legend([cbf, cbf_hat, mrcbf, mrcbf_hat, safe_set], {'$x$, CBF', '$\widehat{x}$, CBF', '$x$, MRCBF', '$\widehat{x}$, MRCBF', '$\mathcal{C}$'}, 'Interpreter', 'latex', 'Location', 'SouthEast')
ylim([-0.5,1.3])
xlim([0, t_max])
xlabel('time [sec]', 'Interpreter', 'latex') 
ylabel('$x \textrm{ [m]}$', 'Interpreter', 'latex')

SAVE_FIG = false;
if SAVE_FIG
    print(gcf,'sim_trajectory.png','-dpng','-r600');
end

%%

figure
hold on 
plot(t_u_mrcbf, u_mrcbf', 'r')
plot(t_u_cbf, u_cbf', 'b')
title("u")

%% Plot Runaround 

t_max = 60; 
bag = rosbag('mrcbf_runaround.bag');

bSel = select(bag,'Topic','/segway_sim/state');
msg_structs = readMessages(bSel, "DataFormat", "struct");
for i = 1:length(msg_structs)
    t_x_runaround(i) = double(msg_structs{i}.Time)/1000000;
    x_runaround(:,i) = msg_structs{i}.StateVec;
    if t_x_runaround(i) >t_max
        break
    end
end
x_runaround = x_runaround';

bSel = select(bag,'Topic','/segway_sim/input');
msg_structs = readMessages(bSel, "DataFormat", "struct");
t0 = double(msg_structs{1}.Header.Stamp.Sec) + 10^-9*double(msg_structs{1}.Header.Stamp.Nsec);
for i = 1:length(msg_structs)
    t_u_runaround(i) = double(msg_structs{i}.Header.Stamp.Sec) + 10^-9*double(msg_structs{i}.Header.Stamp.Nsec) - double(t0);
    u_runaround(:,i) = msg_structs{i}.InputVec;
    if t_u_runaround(i)>t_max
        break
    end
end

bSel = select(bag,'Topic','/segway_sim/inputDes');
msg_structs = readMessages(bSel, "DataFormat", "struct");
t0 = double(msg_structs{1}.Header.Stamp.Sec) + 10^-9*double(msg_structs{1}.Header.Stamp.Nsec);
for i = 1:length(msg_structs)
    t_u_des_runaround(i) = double(msg_structs{i}.Header.Stamp.Sec) + 10^-9*double(msg_structs{i}.Header.Stamp.Nsec) - double(t0);
    u_des_runaround(:,i) = msg_structs{i}.InputVec;
    if t_u_des_runaround(i)>t_max
        break
    end
end


%%
figure
plot(t_x_runaround, x_runaround)

figure
hold on 

plot(t_u_runaround, u_runaround(1,:))
plot(t_u_des_runaround, u_des_runaround(1,:))
legend("u act", "u des")
