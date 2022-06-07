% plots subjects' performance
close all;
clear all;
clc;

for condition = 3:6
    figure('units','normalized','outerposition',[0.15 0.15 0.8 0.8]);
    for subject = 1:10
        switch condition
            case 3
                load(sprintf('Constant/S%d.mat', (condition-1)*10+subject));
                trialConditions = (trialConditions-1)*10; %changing to nominal stiffness values
            case 4
                load(sprintf('Reverse/S%d.mat', (condition-1)*10+subject));
                trialConditions = (trialConditions-1)*10; %changing to nominal stiffness values
            case 5
                load(sprintf('Variable/S%d.mat', (condition-1)*10+subject));
                trialConditions = (trialConditions-1)*10; %changing to nominal stiffness values
            case 6 %%%%%%% MEGHAN'S OLD ELBOW CASE %%%%%
                load(sprintf('Original/S%d.mat', subject));
                trialConditions = trialConditions*10; %changing to nominal stiffness values
        end
        
        %linear fit
        [f,g] = fit(trialConditions', ratings','poly1');
        p = coeffvalues(f); %returns the coefficient values of the cfit object f
        
%         %p-value statistics
%         [rtemp,ptemp] = corrcoef(trialConditions', ratings');
%         pv((condition-1)*10+subject) = ptemp(2);
        
        %plotting
        [uxy, ~, idx] = unique([trialConditions',ratings'],'rows'); %returns unique values
        szscale = histc(idx,unique(idx));
        subplot(2, 5, subject);
        hold on;
        scatter(uxy(:,1),uxy(:,2),'o','filled','sizedata',szscale*50);
        plot(0:10:50, p(1)*(0:10:50)+p(2),'linewidth', 2);
        
        set(gca, 'XLim', [-10 60], 'XTick', 0:10:50, 'YLim', [0 8], 'YTick', 1:7, 'Fontsize', 12.5);
        if condition == 6 || condition == 7 %%%%%%% MEGHAN'S OLD CASE %%%%%
            title(sprintf('R^2 = %.2f', g.rsquare));
            %title(sprintf('R^2 = %.2f, p = %.4f', g.rsquare, pv((condition-1)*10+subject)));
        else
            title(sprintf('Subject %d   R^2 = %.2f', (condition-1)*10+subject, g.rsquare));
            %title(sprintf('Subject %d\n R^2 = %.2f, p = %.4f', (condition-1)*10+subject, g.rsquare, pv((condition-1)*10+subject)));
        end
    end
    
    subplot(2,5,8); x_label = text(0.5, -0.2, 'Elbow Stiffness (Nm/rad)','Units','normalized','HorizontalAlignment', 'center', 'Fontsize', 24); %x_label = text(-220,-1.5, 'Elbow Stiffness (N/rad)','Fontsize', 24);
    subplot(2,5,6); y_label = text(-0.3, 0, 'Subject Stiffness Rating','Units','normalized','Fontsize', 24); %y_label = text(-335,0, 'Subject Stiffness Rating','Fontsize', 24);
    set(y_label, 'Rotation', 90);
    switch condition %title figures
        case 3
            sgtitle('Constant Velocity-Curvature Relationship','Fontsize', 24);
        case 4
            sgtitle('Reverse Velocity-Curvature Relationship','Fontsize', 24);
        case 5
            sgtitle('Variable Velocity-Curvature Relationship','Fontsize', 24);
        case 6
            sgtitle('Original Velocity-Curvature Relationship','Fontsize', 24);
    end
end