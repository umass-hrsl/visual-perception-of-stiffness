function getTwoLinkStates_Experiment1()
    %function used to produce a mat file for the simulations used in Experiement 1
    close all
    clear all
    
    % define arm parameters from Zatsiorsky 2002
    arm.l1 = .2817; % length in meters
    arm.l2 = .2689; % length in meters
    arm.r1 = .1326; % center of mass in meters
    arm.r2 = .1434; % cenetr of mass in meters
    arm.m1 = 2.0438; % mass in kg
    arm.m2 = 1.1749; % mass in kg
    arm.i1 = 0.0168; %arm.m1*(arm.l1*.322)^2; % moment of inertia in kgm^2
    arm.i2 = 0.0078; %arm.m2*(arm.l2*.303)^2; % moment of inertia in kgm^2
    
    %circle parameters for the desired endpoint position
    circle.A = .1; %amplitude
    circle.f = .3; %frequency
    circle.x0 = .1; %x center
    circle.y0 = .3; %y center
    
    %controller parameters
    control.Kx = [500 0; 0 500]; %endpoint stiffness
    control.Bx = [10 0; 0 10]; %endpoint damping
    control.q_desired = [pi/4; pi/4]; %nominal joint position
    
    % set initial state
    state0 = [pi/4, pi/4, 0 0]; % q1, q2, qdot1, qdot2 
    
    % set experiment conditions
    experimentConditions = {'Original','Constant','Reverse','Variable'};
    
    % set trial conditions
    trialConditions = 0:5; %these six trial conditions denote the elbow joint stifness [0:10:50] Nm/rad
    
    %velocity gain for elbow reverse
    K_reverse = [11.5 10.3 7.2 7.0 6.9 6.9];
    %velocity gain for elbow constant
    K_constant = []; %%%%%%%%% THIS NEEDS TO BE FIXED!!!!!!!
    gainConstant = 5.4; %Used when converting time series for the variable conditions
    
    for experiment = 1:length(experimentConditions) %for all 4 experimental conditions
        for trial = 1:length(trialConditions) %for all 6 simulted joint stiffnesses
            
            % determine Kq from trial condition
            control.Kq = getKq(trialConditions(trial), 'elbow'); %elbow, shoulder, both
            
            % simulate the arm motion
            [t,state] = ode45(@(t,y) updateState(t,y,arm, circle, control),[0:.001:65],state0);
            
            % get rid of the first 5 seconds
            if strcmp(experimentConditions{experiment},'Constant')
                remove = find(t<0);
            else
                remove = find(t<5);
            end
            t(remove) = [];
            state(remove, :) = [];
            t = t-t(1);
            
            %add simulated states and time for the given condition.
            twoLinkStates{trial} = [t state];
            
            % get time stamp (tAdjusted) for different velocity conditions
            switch(experimentConditions{experiment})
                case 'Original'
                    [tAdjusted{trial}] = t;
                case 'Constant'
                    [tAdjusted{trial}] = getTimeSeriesConstant(t, state, arm, circle, control, gainConstant);
                case 'Reverse'
                    [tAdjusted{trial}] = getTimeSeriesReverse(t, state, arm, circle, control, K_reverse(trial));
                case 'Variable'
                    [tAdjusted{trial}] = getTimeSeriesVariable(t, state, state0, arm, circle, control, gainConstant);
            end
            
        end
        save([experimentConditions{experiment},'\twoLinkStates_Experiment1_',experimentConditions{experiment},'.mat'], 'twoLinkStates', 'tAdjusted', 'arm', 'circle', 'control', 'state0');
        clear tAdjusted twoLinkStates
    end

end

function [tAdjusted] = getTimeSeriesVariable(t, state, state0, arm, circle, control, K)
    % determine Kq from trial condition
    control.Kq = getKq(5, 'elbow'); %The values 5 and 'elbow' are used as it gets the simulation of elbow having a stiffness of 50 Nm/rad

    % simulate the arm motion of the the simulation of elbow having a stiffness of 50 Nm/rad
    [t,state] = ode45(@(t,y) updateState(t,y,arm, circle, control),[0:.001:65],state0);

    % get rid of the first 5 seconds
    remove = find(t<5);
    t(remove) = [];
    state(remove, :) = [];
    t = t-t(1);
    
    % get x,y position from joint states
    for i = 1:length(state)
        x(:,i) = forwardKinematics(state(i,:), arm); %postion
        xdot(:,i) = Jacobian(state(i,:), arm)*state(i,3:4)';
    end
    x = x'; %postion
    xdot = xdot'; %velocity
    xdotdot = [.1 .1; diff(xdot)./diff(t)]; %get acceleration
    
    %get time series based on arc length
    arcLength = [0; sqrt(diff(x(:,1)).^2+diff(x(:,2)).^2)];
    tAdjusted = cumsum(K*arcLength);
   
end

function [tAdjusted] = getTimeSeriesConstant(t, state, arm, circle, control, K)

    b = 0; %power lower coefficient

    % get x,y position from joint states
    for i = 1:length(state)
        x(:,i) = forwardKinematics(state(i,:), arm); %postion
        xdot(:,i) = Jacobian(state(i,:), arm)*state(i,3:4)'; %velocity
    end
    
    x = x'; %postiion
    xdot = xdot'; %velocity
    xdotdot = [.1 .1; diff(xdot)./diff(t)]; %get acceleration
    
    arcLength = [0; sqrt(diff(x(:,1)).^2+diff(x(:,2)).^2)];
    roc = abs(((xdot(:,1).^2+xdot(:,2).^2).^(3/2))./(xdot(:,1).*xdotdot(:,2)-xdot(:,2).*xdotdot(:,1))); %radius of curvature
    
    %get time series based on radius of curvature using the power law equation
    tAdjusted = 5.4*arcLength./(roc.^b); %gain K is used to make sure period similar across condition
    tAdjusted(find(~isfinite(tAdjusted))) = .0001; %only the first two terms which don't get shown anyway.
    tAdjusted = cumsum(tAdjusted);
end

function [tAdjusted] = getTimeSeriesReverse(t, state, arm, circle, control, K)

    b = -1/3; %power lower coefficient

    % get x,y position from joint states
    for i = 1:length(state)
        x(:,i) = forwardKinematics(state(i,:), arm); %postion
        xdot(:,i) = Jacobian(state(i,:), arm)*state(i,3:4)'; %velocity
    end
    
    x = x';
    xdot = xdot';
    xdotdot = [.1 .1; diff(xdot)./diff(t)]; %acceleration
    
    arcLength = [0; sqrt(diff(x(:,1)).^2+diff(x(:,2)).^2)];
    roc = abs(((xdot(:,1).^2+xdot(:,2).^2).^(3/2))./(xdot(:,1).*xdotdot(:,2)-xdot(:,2).*xdotdot(:,1))); %radius of curvature
    
    %get time series based on radius of curvature using the power law equation
    tAdjusted = K*arcLength./(roc.^b); %gain K is used to make sure period similar across condition
    tAdjusted(find(~isfinite(tAdjusted))) = .0001; %only the first two terms which don't get shown anyway.
    tAdjusted = cumsum(tAdjusted);
end
    
function der = updateState(time, state, arm, circle, control)

    % simulate motion of two-link planar arm in vertical plane

    % get torgue command from stiffness controller
    Q = getTorqueCommand(state, time, arm, circle, control);
    
    % inertial forces
    M = zeros(2);
    M(1) = arm.i1 + arm.i2 + arm.m1*arm.r1^2 + arm.m2*(arm.l1^2 + arm.r2^2 + 2*arm.l1*arm.r2*cos(state(2)));
    M(2:3) = arm.i2 + arm.m2*(arm.r2^2 + arm.l1*arm.r2*cos(state(2)));
    M(4) = arm.i2 + arm.m2*arm.r2^2;
  
    % coriolis forces
    C = zeros(2,1);
    C(1) = -(arm.m2*arm.l1*state(4)^2*arm.r2*sin(state(2))) - (2*arm.m2*arm.l1*state(3)*state(4)*arm.r2*sin(state(2)));
	C(2) = arm.m2*arm.l1*arm.r2*sin(state(2))*state(3)^2;
    
    % gravitational forces
    G = zeros(2,1);
    g=9.81;
    G(1) = g*arm.m1*arm.r1*cos(state(1)) + arm.m2*g*(arm.l1*cos(state(1))+arm.r2*cos(sum(state(1:2))));
    G(2) = g*arm.m2*arm.r2*cos(sum(state(1:2)));
    
    % rearrange to solve for acceleration
    qdotdot = inv(M)*(Q-C-G);
    
    % solution
    der = [state(3:4); qdotdot];
end

function [torqueCommand] = getTorqueCommand(state, time, arm, circle, control)

    x_current = forwardKinematics(state(1:2),arm);
    v_current = Jacobian(state,arm)*state(3:4);
    x_desired = getXDesired(circle, time);
    q_current = state(1:2);
    
    % force pulling endpoint to desired location
    cartesianForce = control.Kx*(x_desired - x_current)-control.Bx*v_current;
    % tranform force to torque
    cartesianTorque = JacobianT(state,arm)*cartesianForce;
    % torque pulling joints to desired location
    jointTorque = control.Kq*(control.q_desired - q_current);
    % resulting joint commands
    torqueCommand = cartesianTorque+jointTorque;
    
end

function x = forwardKinematics(state, arm)
    % get endpoint position from joint angles
    x = [arm.l1*cos(state(1)) + arm.l2*cos(sum(state(1:2)));
         arm.l1*sin(state(1)) + arm.l2*sin(sum(state(1:2)))];
end

function J = Jacobian(state, arm) %jacobian
    J = [-arm.l1*sin(state(1))-arm.l2*sin(sum(state(1:2))) -arm.l2*sin(sum(state(1:2)));
         arm.l1*cos(state(1))+arm.l2*cos(sum(state(1:2))) arm.l2*cos(sum(state(1:2)))];
end


function JT = JacobianT(state,arm) %jacobian transpose
    JT = [-arm.l1*sin(state(1))-arm.l2*sin(sum(state(1:2))) arm.l1*cos(state(1))+arm.l2*cos(sum(state(1:2)));
         -arm.l2*sin(sum(state(1:2))) arm.l2*cos(sum(state(1:2)))];
end

function x = getXDesired(circle, time) %desired endpoint position
    x = [circle.A*cos(2*pi*circle.f*time)+circle.x0; circle.A*sin(2*pi*circle.f*time)+circle.y0];   
end

function Kq = getKq(stiffnessCondition, joint) %get simulated joint stiffness based on the joint and the stiffness condition

    switch(joint)
        case 'elbow'
            index = 4;
        case 'shoulder' 
            index = 1;
        case 'both'
            index = [1 4];
    end
            

    Kq = zeros(2,2);
    
    switch(stiffnessCondition)
        case 1
            Kq(index) = 10;
        case 2
            Kq(index) = 20;
        case 3 
            Kq(index) = 30;
        case 4
            Kq(index) = 40;
        case 5
            Kq(index) = 50;
    end
end
