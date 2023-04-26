%Moment arms (in m). Order: 
% 1. hip, 2. knee, 3. ankle in columns
MomentArms=[3.4 0 0; ... iliacus
    4.2 4.3 0; ... rectus femoris
    -5.6 -3.3 0;... biceps femoris
    -4.8 -4.0 0;... semitendinosus
    -6.0 0 0; ... gluteus maximus
    0 3.9 0;... vastus lateralis
    0 3.8 0;... vastus medialis
    0 -1.8 -4.7;... gastrocnemius lateralis
    0 0 -4.5;... soleus
    0 0 4.2]*0.01; % tibialis anterior

% Maximum isometric forces. Same order as the moment arms
Fiso=[1073; 1169; 804; 1288; 1000; 1871; 1294; 683; 3549; 905];
% Fiso(6)=Fiso(6)*0.5;
% Fiso(7)=Fiso(7)*0.5;

% Given moments at the hip, knee and ankle
M=[-60; 80; -100];

%% quadprog (from quadratic programming) in MATLAB solves a quadratic programming problem
% One can call it as follows: X = quadprog(H,f,A,b,Aeq,beq,LB,UB,X0,OPTIONS)
% It solves the following problem:
% min 0.5*x'*H*x + f'*x   subject to:  A*x <= b 

% So, in our case, x are the muscle activations (10 muscle activations), 
% which are our design variables (variables to be optimized), 
% H=2*ones(nmuscles, nmuscles), f=zeros(nmuscles,1), A and b do not apply, 
% since we do not have inequality constraints in our problem; and 
% Aeq will be the moment arms and beq the given joint moments

nmuscles=10; %we have 10 muscles
H=2*eye(nmuscles,nmuscles); 
f=zeros(nmuscles,1);
A=[];
b=[];
Aeq=(MomentArms').*Fiso'; %note that x is a vector of 10 rows, so 
    % MomentArms variable needs to be transposed
beq=M;
%muscle activations are bound between 0 (lower bound - lb) and 1 (upper
%bound - 1)
lb=zeros(nmuscles,1);
ub=ones(nmuscles,1);

%Solve the problem
sol=quadprog(H,f,A,b,Aeq,beq,lb,ub);

% Calculate joint moments from the solution to check if the statement of
% the problem is satisfied
Moment_sol=sum(MomentArms.*sol.*Fiso);

%% Plot muscle activations
figure
muscle_labels={'ili','rf','bf','semim','glumax','vaslat','vasmed','gaslat','sol','tibant'};
bar(sol);
set(gca,'XTick',[1:10],'XTickLabels', muscle_labels);
ylabel('muscle activations []');
title('Muscle activations');

%% Plot muscle forces
figure
muscle_labels={'ili','rf','bf','semim','glumax','vaslat','vasmed','gaslat','sol','tibant'};
bar(sol.*Fiso);
set(gca,'XTick',[1:10],'XTickLabels', muscle_labels);
ylabel('muscle forces []');
title('Muscle forces');

%% Resultant joint moments
figure
joint_moments={'hip','knee','ankle'};
bar(Moment_sol);
set(gca,'XTick',[1:10],'XTickLabels', joint_moments);
ylabel('Joint Moments [Nm]');
title('Joint moments');