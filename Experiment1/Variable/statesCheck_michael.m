%Aaron Michael West Jr
%07/16/2019
%code to check the EWeird_new.mat states

clear all
close all
clc

load('twoLinkStatesEWeird_new.mat');
figure('units','normalized','outerposition',[0 0 1 1]);
figure('units','normalized','outerposition',[0 0 1 1]);
for condition = 1:6
    x1 = arm.l1*cos(twoLinkStates{1,condition}(:,2));
    y1 = arm.l1*sin(twoLinkStates{1,condition}(:,2));
    
    x2 = arm.l1*cos(twoLinkStates{1,condition}(:,2))+arm.l2*cos(twoLinkStates{1,condition}(:,2)+twoLinkStates{1,condition}(:,3));
    y2 = arm.l1*sin(twoLinkStates{1,condition}(:,2))+arm.l2*sin(twoLinkStates{1,condition}(:,2)+twoLinkStates{1,condition}(:,3));
    
    for i = 1:length(x2)
    DOT(:,i) = [-arm.l1*sin(twoLinkStates{1,condition}(i,2)) -arm.l2*sin(twoLinkStates{1,condition}(i,2)+twoLinkStates{1,condition}(i,3));
        arm.l1*cos(twoLinkStates{1,condition}(i,2)) arm.l2*cos(twoLinkStates{1,condition}(i,2)+twoLinkStates{1,condition}(i,3))]*[1 0; 1 1]*[twoLinkStates{1,condition}(i,4);twoLinkStates{1,condition}(i,5)];
    end
    x2DOT = DOT(1,:)';
    y2DOT = DOT(2,:)';
%     figure;
%     for i = 1:length(x1)
%         plot([0 x1(i)],[0 y1(i)],'b','linewidth',2); hold on;
%         plot([x1(i) x2(i)],[y1(i) y2(i)],'b','linewidth',2); hold on;
%         plot(x2(1:i),y2(1:i),'r-o','LineWidth',2); hold on;
%         pause(0.00001);
%         hold off;
%     end
    figure(1); plot(x2,y2,'linewidth',2); axis equal; hold on;
    
    [ROC, vel_tang] = getROC(tConstant,x2,y2,x2DOT,y2DOT);
    figure(2); subplot(2,3, condition); loglog(ROC, vel_tang,'o');
    figure(2); subplot(2,3, condition); xlabel('log ROC(m)'); ylabel('log tan vel (m/s)');
    E = (condition-1)*10;
    figure(2); subplot(2,3, condition); title(['E = ',num2str(E),' Nm']);
end
figure(1); title('Elbow'); xlabel('X(m)'); ylabel('Y(m)');
figure(1); legend('E = 0','E = 10','E = 20','E = 30','E = 40','E = 50');

function [ROC, vel_tang] = getROC(tConst,x,y,xDOT,yDOT)
    xDOTDOT = [0; diff(xDOT)./diff(tConst)];
    yDOTDOT = [0; diff(yDOT)./diff(tConst)];
    
    kappa = abs(xDOT.*yDOTDOT-yDOT.*xDOTDOT)./((xDOT.^2+yDOT.^2).^1.5);
    ROC = (kappa(2:end)).^-1;
    
    arcLength = [0; sqrt(diff(x).^2+diff(y).^2)];
    vel_tang = arcLength(2:end)./(diff(tConst));
end