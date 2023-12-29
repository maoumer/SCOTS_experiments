%
% diff_drive_smooth.m
%
%  created on: 22.11.2023
%     author: mohammed adib oumer
%
% see readme file for more information on the diff_drive_smooth example
%
% you need to run ./diff_drive_smooth binary first 
%
% so that the files: diff_drive_smooth_ss.bdd 
%                    diff_drive_smooth_obst.bdd
%                    diff_drive_smooth_target.bdd
%                    diff_drive_smooth_controller.bdd 
% are created
%

function diff_drive_smooth
clear set
close all

%% simulation

% target set
lb=[9 0];
ub=lb+0.5;
v=[9 0; 9.5  0; 9 0.5; 9.5 .5];
% initial state (5 state system -> 5 entries)
x0= [4 2 -0.6000  -49.9999  -83.3333];



controller=SymbolicSet('diff_drive_smooth_controller.bdd','projection',[1 2 3 4 5]);
target=SymbolicSet('diff_drive_smooth_target.bdd');

domain_points = controller.points;  
plot(domain_points(:,1),domain_points(:,2),'.');
% find(domain_points(:,1)==3, 1)
% x0 = domain_points(find(domain_points(:,1)==2.4, 1, 'first'),:)

% sample one random row from feasible region and set as initial state
% y=[domain_points(randperm(length(domain_points),1),:)];

% get a leftmost point in the domain
[~,I] = max(domain_points(:,2));
x0=domain_points(I,:);
y=x0;% - originally
v=[];

while(1)

  if (target.isElement(y(end,:))) 
    break;
  end 

  u=controller.getInputs(y(end,:));
  v=[v; u(1,:)];
  [t x]=ode45(@ode_equations,[0 .3], y(end,:), [],u(1,:));

  y=[y; x(end,:)];
end



%% plot the diff_drive_smooth domain
% colors
colors=get(groot,'DefaultAxesColorOrder');


% load the symbolic set containing the abstract state space
set=SymbolicSet('diff_drive_smooth_ss.bdd','projection',[1 2]);
plotCells(set,'facecolor','none','edgec',[0.8 0.8 0.8],'linew',.1)
hold on

% load the symbolic set containig obstacles
% set=SymbolicSet('diff_drive_smooth_obst.bdd','projection',[1 2]);
% plotCells(set,'facecolor',colors(1,:)*0.5+0.5,'edgec',colors(1,:),'linew',.1)

% plot the real obstacles and target set
% plot_domain

% load the symbolic set containig target set
set=SymbolicSet('diff_drive_smooth_target.bdd','projection',[1 2]);
plotCells(set,'facecolor',colors(2,:)*0.5+0.5,'edgec',colors(2,:),'linew',.1)

% plot initial state  and trajectory
plot(y(:,1),y(:,2),'k.-')
plot(y(1,1),y(1,1),'.','color',colors(5,:),'markersize',20)


box on
axis([-.4 4.2 -.4 4.2])

saveas(gcf,'simulation.png')

end

function dxdt = ode_equations(t,x,u)

  dxdt = zeros(5,1);
  rad = 0.06; L = 0.4;

  dxdt(1)=(rad/2.0)*(x(5)+x(4))*cos(x(3));
  dxdt(2)=(rad/2.0)*(x(5)+x(4))*sin(x(3));
  dxdt(3)=(rad/L)*(x(5)-x(4));
  dxdt(4)=u(1);
  dxdt(5)=u(2);

end

function plot_domain

colors=get(groot,'DefaultAxesColorOrder');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%NEW%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Targets
v=[4.2 0; 5  0; 4.2 0.7; 5 .7];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(2,:),'edgec',colors(2,:));

% Obstacles
v=[1.61     1 ;2  1   ; 1.61     4    ; 2 4   ];  %{-1.61,2,-1, 4};
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
v=[3.1  0  ;3.3  0   ; 3.1   3   ; 3.3  3  ]; %{-3.21,3.6,-0,2};
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));



end
