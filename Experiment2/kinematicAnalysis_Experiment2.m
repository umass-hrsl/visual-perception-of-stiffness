%Michael West
%11/23/2021
%Analysis on kinematics of motion of Experiment 2.
%This code helped produce Figure 13

clear all; close all; clc;

% load pre-computed simulation data
load('twoLinkStates_Experiment2.mat'); %loads the variables arm, circle, control, state0, tAdjusted, twoLinkStates
%tAdjusted is the time trajectory used in the actual simulation.
%twoLinkStates{i}(:,2:end) are the states (joint angles, and velocities.

%RGB colors from paper
colorOrder_RGB = [146 186 190;...% Original (Teal)
              140 24 27;... %Constant (Dark Red)
              237 28 36;... %Reverse (Red)
              241 102 103;]; %Absent (Rose)
colorOrder_RGB = colorOrder_RGB./255;


fig1 = figure('units','normalized','outerposition',[0.4 0.1 0.6 0.95]);
for velocityProfile = 1:4
    co = colorOrder_RGB(velocityProfile,:);
    if velocityProfile == 1
        qdot{velocityProfile} = twoLinkStates{velocityProfile}(:,3:4);
    else
        qdot{velocityProfile} = [0.1, 0.1; diff(twoLinkStates{velocityProfile}(:,1:2))./diff(tAdjusted{velocityProfile}(:,1))];
    end
    qdotdot{velocityProfile} = [0.1, 0.1; diff(qdot{velocityProfile}(:,1:2))./diff(tAdjusted{velocityProfile}(:,1))];
    qdotdotdot{velocityProfile} = [0.1, 0.1; diff(qdotdot{velocityProfile}(:,1:2))./diff(tAdjusted{velocityProfile}(:,1))];
    for i = 1:length(tAdjusted{velocityProfile})
        state = [twoLinkStates{velocityProfile}(i,2:3)'; qdot{velocityProfile}(i,1:2)'];
        time(i) = tAdjusted{velocityProfile}(i);
%         if time(i) > 22
%             break
%         end

%         sumqdot{velocityProfile}(i,1) = sum(qdot{velocityProfile}(i,:));
%         sumqdotdot{velocityProfile}(i,1) = sum(qdotdot{velocityProfile}(i,:));
%         sumqdotdotdot{velocityProfile}(i,1) = sum(qdotdotdot{velocityProfile}(i,:));
        [x_actual{velocityProfile}(i,:), v_actual{velocityProfile}(i,:)] = forwardKinematics_ALL(state, arm);
    end
    %differentiate to get accel and jerk
    accel_actual{velocityProfile} = [0.1, 0.1; diff(v_actual{velocityProfile})./diff(tAdjusted{velocityProfile}(:,1))];
    jerk_actual{velocityProfile} = [0.1, 0.1; diff(accel_actual{velocityProfile})./diff(tAdjusted{velocityProfile}(:,1))];
    
    %find peaks using function I created to get rms for 2 cycle
    [pks_Edot(velocityProfile,:), locs_Edot(velocityProfile,:)] = findpeaks_michael(qdot{velocityProfile}(:,1));
    [pks_Edotdot(velocityProfile,:), locs_Edotdot(velocityProfile,:)] = findpeaks_michael(qdotdot{velocityProfile}(:,1));
    [pks_Edotdotdot(velocityProfile,:), locs_Edotdotdot(velocityProfile,:)] = findpeaks_michael(qdotdotdot{velocityProfile}(:,1));
    [pks_Sdot(velocityProfile,:), locs_Sdot(velocityProfile,:)] = findpeaks_michael(qdot{velocityProfile}(:,2));
    [pks_Sdotdot(velocityProfile,:), locs_Sdotdot(velocityProfile,:)] = findpeaks_michael(qdotdot{velocityProfile}(:,2));
    [pks_Sdotdotdot(velocityProfile,:), locs_Sdotdotdot(velocityProfile,:)] = findpeaks_michael(qdotdotdot{velocityProfile}(:,2));
    %get time of the last peak so we can plot just two cycles later on
    t_Edot(velocityProfile,:) = time(locs_Edot(velocityProfile,:));
    t_Edotdot(velocityProfile,:) = time(locs_Edotdot(velocityProfile,:));
    t_Edotdotdot(velocityProfile,:) = time(locs_Edotdotdot(velocityProfile,:));
    t_Sdot(velocityProfile,:) = time(locs_Sdot(velocityProfile,:));
    t_Sdotdot(velocityProfile,:) = time(locs_Sdotdot(velocityProfile,:));
    t_Sdotdotdot(velocityProfile,:) = time(locs_Sdotdotdot(velocityProfile,:));
    
    %find peaks using function I created to get rms for 1 cycle
    [pks_Vx(velocityProfile,:), locs_Vx(velocityProfile,:)] = findpeaks_michael(v_actual{velocityProfile}(:,1));
    [pks_Ax(velocityProfile,:), locs_Ax(velocityProfile,:)] = findpeaks_michael(accel_actual{velocityProfile}(:,1));
    [pks_Jx(velocityProfile,:), locs_Jx(velocityProfile,:)] = findpeaks_michael(jerk_actual{velocityProfile}(:,1));
    [pks_Vy(velocityProfile,:), locs_Vy(velocityProfile,:)] = findpeaks_michael(v_actual{velocityProfile}(:,2));
    [pks_Ay(velocityProfile,:), locs_Ay(velocityProfile,:)] = findpeaks_michael(accel_actual{velocityProfile}(:,2));
    [pks_Jy(velocityProfile,:), locs_Jy(velocityProfile,:)] = findpeaks_michael(jerk_actual{velocityProfile}(:,2));
    
    %RMS
    rms_qdot(:,velocityProfile) =       [rms(qdot{velocityProfile}(locs_Edot(velocityProfile,1):locs_Edot(velocityProfile,2),1)); rms(qdot{velocityProfile}(locs_Sdot(velocityProfile,1):locs_Sdot(velocityProfile,2),2))];
    rms_qdotdot(:,velocityProfile) =    [rms(qdotdot{velocityProfile}(locs_Edotdot(velocityProfile,1):locs_Edotdot(velocityProfile,2),1)); rms(qdotdot{velocityProfile}(locs_Sdotdot(velocityProfile,1):locs_Sdotdot(velocityProfile,2),2))];
    rms_qdotdotdot(:,velocityProfile) = [rms(qdotdotdot{velocityProfile}(locs_Edotdotdot(velocityProfile,1):locs_Edotdotdot(velocityProfile,2),1)); rms(qdotdotdot{velocityProfile}(locs_Sdotdotdot(velocityProfile,1):locs_Sdotdotdot(velocityProfile,2),2))];
    rms_V(:,velocityProfile) = [rms(v_actual{velocityProfile}(locs_Vx(velocityProfile,1):locs_Vx(velocityProfile,2),1)); rms(v_actual{velocityProfile}(locs_Vy(velocityProfile,1):locs_Vy(velocityProfile,2),2))];
    rms_A(:,velocityProfile) = [rms(accel_actual{velocityProfile}(locs_Ax(velocityProfile,1):locs_Ax(velocityProfile,2),1)); rms(accel_actual{velocityProfile}(locs_Ay(velocityProfile,1):locs_Ay(velocityProfile,2),2))];
    rms_J(:,velocityProfile) = [rms(jerk_actual{velocityProfile}(locs_Jx(velocityProfile,1):locs_Jx(velocityProfile,2),1)); rms(jerk_actual{velocityProfile}(locs_Jy(velocityProfile,1):locs_Jy(velocityProfile,2),2))];
    
    %plotting
    n = length(qdot{velocityProfile});
    %plot rms on bar plot
    figure(fig1);
    subplot(3,4,1); hold on; bar(velocityProfile,rms_qdot(1,velocityProfile),'FaceColor',co); ylim([0 4]);
    subplot(3,4,2); hold on; bar(velocityProfile,rms_qdot(2,velocityProfile),'FaceColor',co); ylim([0 4]);
    subplot(3,4,5); hold on; bar(velocityProfile,rms_qdotdot(1,velocityProfile),'FaceColor',co); set(gca,'YScale','log'); ylim([1e-1 1.5e2]);
    subplot(3,4,6); hold on; bar(velocityProfile,rms_qdotdot(2,velocityProfile),'FaceColor',co); set(gca,'YScale','log'); ylim([1e-1 1.5e2]);
    subplot(3,4,9);  hold on; bar(velocityProfile,rms_qdotdotdot(1,velocityProfile),'FaceColor',co); set(gca,'YScale','log'); ylim([1e-1 1e5]);
    subplot(3,4,10); hold on; bar(velocityProfile,rms_qdotdotdot(2,velocityProfile),'FaceColor',co); set(gca,'YScale','log'); ylim([1e-1 1e5]);
    %pause;

    %plot rms on bar plot
    figure(fig1);
    subplot(3,4,3); hold on; bar(velocityProfile,rms_V(1,velocityProfile),'FaceColor',co); ylim([0 4]);
    subplot(3,4,4); hold on; bar(velocityProfile,rms_V(2,velocityProfile),'FaceColor',co); ylim([0 4]);
    subplot(3,4,7); hold on; bar(velocityProfile,rms_A(1,velocityProfile),'FaceColor',co); set(gca,'YScale','log'); ylim([1e-1 1.5e2]);
    subplot(3,4,8); hold on; bar(velocityProfile,rms_A(2,velocityProfile),'FaceColor',co); set(gca,'YScale','log'); ylim([1e-1 1.5e2]);
    subplot(3,4,11); hold on; bar(velocityProfile,rms_J(1,velocityProfile),'FaceColor',co); set(gca,'YScale','log'); ylim([1e-1 1e5]);
    subplot(3,4,12); hold on; bar(velocityProfile,rms_J(2,velocityProfile),'FaceColor',co); set(gca,'YScale','log'); ylim([1e-1 1e5]);
    %pause;
end
%labelling
%joint space
subplot(3,4,1); ylabel('Velocity (rad/sec)'); title('RMS Shoulder'); set(gca, 'fontsize', 14); xticks(1:4); xticklabels({'Original','Constant','Reverse','Absent'}); xtickangle(45);
subplot(3,4,2); ylabel('Velocity (rad/sec)'); title('RMS Elbow'); set(gca, 'fontsize', 14); xticks(1:4); xticklabels({'Original','Constant','Reverse','Absent'}); xtickangle(45);
subplot(3,4,5); ylabel('Acceleration (rad/sec^2)'); set(gca, 'fontsize', 14); xticks(1:4); xticklabels({'Original','Constant','Reverse','Absent'}); xtickangle(45);
yticks(logspace(-1,2,4)); yticklabels({'10^{-1}','10^{0}','10^{1}','10^{2}',});
subplot(3,4,6); ylabel('Acceleration (rad/sec^2)'); set(gca, 'fontsize', 14); xticks(1:4); xticklabels({'Original','Constant','Reverse','Absent'}); xtickangle(45);
yticks(logspace(-1,2,4)); yticklabels({'10^{-1}','10^{0}','10^{1}','10^{2}',});
subplot(3,4,9); ylabel('Jerk (rad/sec^3)'); set(gca, 'fontsize', 14); xticks(1:4); xticklabels({'Original','Constant','Reverse','Absent'}); xtickangle(45);
yticks(logspace(-1,5,4)); yticklabels({'10^{-1}','10^{1}','10^{3}','10^{5}',});
subplot(3,4,10); ylabel('Jerk (rad/sec^3)'); set(gca, 'fontsize', 14); xticks(1:4); xticklabels({'Original','Constant','Reverse','Absent'}); xtickangle(45);
yticks(logspace(-1,5,4)); yticklabels({'10^{-1}','10^{1}','10^{3}','10^{5}',});
%cartesian space
subplot(3,4,3); ylabel('Velocity (m/sec)'); title('RMS X-Coordinate'); set(gca, 'fontsize', 14); xticks(1:4); xticklabels({'Original','Constant','Reverse','Absent'}); xtickangle(45);
subplot(3,4,4); ylabel('Velocity (m/sec)'); title('RMS Y-Coordinate'); set(gca, 'fontsize', 14); xticks(1:4); xticklabels({'Original','Constant','Reverse','Absent'}); xtickangle(45);
subplot(3,4,7); ylabel('Acceleration (m/sec^2)'); set(gca, 'fontsize', 14); xticks(1:4); xticklabels({'Original','Constant','Reverse','Absent'}); xtickangle(45);
yticks(logspace(-1,2,4)); yticklabels({'10^{-1}','10^{0}','10^{1}','10^{2}',});
subplot(3,4,8); ylabel('Acceleration (m/sec^2)'); set(gca, 'fontsize', 14); xticks(1:4); xticklabels({'Original','Constant','Reverse','Absent'}); xtickangle(45);
yticks(logspace(-1,2,4)); yticklabels({'10^{-1}','10^{0}','10^{1}','10^{2}',});
subplot(3,4,11); ylabel('Jerk (m/sec^3)'); set(gca, 'fontsize', 14); xticks(1:4); xticklabels({'Original','Constant','Reverse','Absent'}); xtickangle(45);
yticks(logspace(-1,5,4)); yticklabels({'10^{-1}','10^{1}','10^{3}','10^{5}',});
subplot(3,4,12); ylabel('Jerk (m/sec^3)'); set(gca, 'fontsize', 14); xticks(1:4); xticklabels({'Original','Constant','Reverse','Absent'}); xtickangle(45);
yticks(logspace(-1,5,4)); yticklabels({'10^{-1}','10^{1}','10^{3}','10^{5}',});
%saveas(gcf,'kinematicsBarGraphs_ControlExperiment.png');

%% Internally Used Functions
function [torqueFwdPath, x_desired] = getTorqueFwdPath(state, torqueCommand, time, arm, circle, control)

    x_current = forwardKinematics_ALL(state(1:2),arm);
    v_current = Jacobian(state,arm)*state(3:4);
    q_current = state(1:2);
    
    % torque pulling joints to desired location
    jointTorque = control.Kq*(control.q_desired - q_current);
    
    torqueFwdPath = torqueCommand + JacobianT(state,arm)*control.Kx*x_current + JacobianT(state,arm)*control.Bx*v_current - jointTorque;
    
    %get the zero force trajectory
    x_desired = inv(control.Kx)*inv(JacobianT(state,arm))*torqueFwdPath;
end

function [x, v] = forwardKinematics_ALL(state, arm)
    % get endpoint position from joint angles
    x = [arm.l1*cos(state(1)) + arm.l2*cos(sum(state(1:2)));
         arm.l1*sin(state(1)) + arm.l2*sin(sum(state(1:2)))];
     
    v = Jacobian(state,arm)*state(3:4); %velocity
    
%     arcLength = [0; sqrt(diff(x(:,1)).^2+diff(x(:,2)).^2)];
%     tanVel = arcLength(2:end)./(diff(tAdjusted));
%     tanAcc = [0; diff(tanVel)./diff(tAdjusted(2:end))];
%     tanJerk = [0; diff(tanAcc)./diff(tAdjusted(2:end))];
    
end

function [tanVel, tanAcc, tanJerk] = getTangentialKinematics(x,t)
    arcLength = [0; sqrt(diff(x(:,1)).^2+diff(x(:,2)).^2)];
    tanVel = [0; arcLength(2:end)./(diff(t))];
    tanAcc = [0; 0; diff(tanVel(2:end))./diff(t(2:end))];
    tanJerk = [0; 0; diff(tanAcc(2:end))./diff(t(2:end))];
end

function J = Jacobian(state, arm) 
    J = [-arm.l1*sin(state(1))-arm.l2*sin(sum(state(1:2))) -arm.l2*sin(sum(state(1:2)));
         arm.l1*cos(state(1))+arm.l2*cos(sum(state(1:2))) arm.l2*cos(sum(state(1:2)))];
end


function JT = JacobianT(state,arm)
    JT = [-arm.l1*sin(state(1))-arm.l2*sin(sum(state(1:2))) arm.l1*cos(state(1))+arm.l2*cos(sum(state(1:2)));
         -arm.l2*sin(sum(state(1:2))) arm.l2*cos(sum(state(1:2)))];
end

function x = getXDesired(circle, time)
    x = [circle.A*cos(2*pi*circle.f*time)+circle.x0; circle.A*sin(2*pi*circle.f*time)+circle.y0];   
end

function [pks, locs] = findpeaks_michael(vec)
    chop_ = 6; %where to chop the vector to remove the transients
    %chop the initial transients
    vec2 = vec(chop_:end);
    
    [pks, loc] = findpeaks(vec2,'MinPeakDistance',2e3);
%     figure; plot(vec2); hold on; plot(loc, pks,'x');
%     pause;
    
    % return values in orginal vector
    locs = [loc(2)+chop_-1,loc(3)+chop_-1]; %by using loc(2) and loc(3) here I have chosen to look at solely the second cycle
    pks = vec(locs);
    
%     %check by plotting
%     figure; plot(chop_:length(vec),vec(chop_:end),'-',locs,pks,'rx'); %xlim([chop_ length(vec)]);
%     xlim([chop_ locs(2)+50]);
%     pause;
end