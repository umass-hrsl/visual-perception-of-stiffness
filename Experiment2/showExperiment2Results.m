%to look at all subject data in Experiment 2
%this code helped produce Figure 11
clear all;
clc;
close all;


fig = figure('units','normalized','outerposition',[0.25 0.25 0.75 0.75]);
i = 1;
for si = 51:60 %loop through the 10 subjects
    load(sprintf('S%d.mat', si)); %load the MAT files
    for j = 1:20 %loop to find repetitions of a simulation rating pair
        test = [trialConditions == trialConditions(j);ratings == ratings(j)];
        dex = find(trialConditions == trialConditions(j) & ratings == ratings(j));
        markerSizeString(1,j) = length(dex); %increase markersize based on frequency of that simulation rating pair
    end
    subplot(2,5,i); ax = gca;
    scatter(trialConditions,ratings,50+50*markerSizeString,'filled'); hold on;
    title(['Subject ',int2str(si)],'fontsize',16); ylim([1 7]); xlim([0.5 4.5]); grid on;
    ax.XLabel = []; ax.YLabel = [];
    ax.XTickLabelMode = 'manual';
    ax.XTick = 1:4;
    ax.XTickLabelRotation = 45;
    ax.XTickLabel = {'Original','Constant','Reverse','Absent'};
    
    i = i+1;
end
% Give common xlabel, ylabel and title to your figure
han=axes(fig,'visible','off'); 
han.Title.Visible='on';
han.XLabel.Visible='on';
han.YLabel.Visible='on';
ylabel(han,'Stiffness Rating','fontsize',24);
xlabel(han,{[];['Velocity Profile']},'fontsize',24);
title(han,{['Simulated Elbow Stiffness = 30Nm/rad'];[]},'fontsize',24);
%saveas(gcf,'controlExperimentResults.png');