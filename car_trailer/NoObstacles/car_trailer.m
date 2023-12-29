%
% car_trailer.m
%
% created on: 05.12.2023
%     author: mohammed adib oumer
%
% see readme file for more information on the car_trailer example
%
% you need to run ./car_trailer binary first 
%
% so that the files: car_trailer_ss.bdd 
%                    car_trailer_obst.bdd
%                    car_trailer_target.bdd
%                    car_trailer_controller.bdd 
% are created
%

function car_trailer
clear set
close all


%% simulation

% target set
lb=[9 0];
ub=lb+0.5;
v=[9 0; 9.5  0; 9 0.5; 9.5 .5];
% initial state (4 state system -> 4 entries)
x0=[0 10 0 0]; % must be within controller domain. See other ways of generating x0 below

controller=SymbolicSet('car_trailer_controller.bdd','projection',[1 2 3 4]);
target=SymbolicSet('car_trailer_target.bdd');

domain_points = controller.points;
plot(domain_points(:,1),domain_points(:,2),'.')
% domain_points(find(domain_points(:,1)==6.8, 1, 'first'),:)

% sample one random row from feasible region and set as initial state
% x0=[domain_points(randperm(length(domain_points),1),:)];

% get a leftmost point in the domain
% [~,I] = min(domain_points(:,1));
% x0=domain_points(I,:)

y=x0;% - originally
v=[];

while(1)

  
  if (target.isElement(y(end,:))) 
    break;
  end 

  u=controller.getInputs(y(end,:)); % potentially multiple input options produced here
  v=[v; u(end,:)]; % u can use the first, last or any of the input options from line 56. The 3 images show the different results because of this
  [t x]=ode45(@unicycle_ode,[0 .3], y(end,:), [], u(end,:)); % same comment as line 57

  y=[y; x(end,:)];
end



%% plot the car_trailer domain
% colors
colors=get(groot,'DefaultAxesColorOrder');


% load the symbolic set containing the abstract state space
set=SymbolicSet('car_trailer_ss.bdd','projection',[1 2]);
plotCells(set,'facecolor','none','edgec',[0.8 0.8 0.8],'linew',.1)
hold on

% load the symbolic set containig obstacles
% set=SymbolicSet('car_trailer_obst.bdd','projection',[1 2]);
% plotCells(set,'facecolor',colors(1,:)*0.5+0.5,'edgec',colors(1,:),'linew',.1)

% plot the real obstacles and target set
plot_domain

% load the symbolic set containig target set
set=SymbolicSet('car_trailer_target.bdd','projection',[1 2]);
plotCells(set,'facecolor',colors(2,:)*0.5+0.5,'edgec',colors(2,:),'linew',.1)

% plot initial state  and trajectory
plot(y(:,1),y(:,2),'k.-')
plot(y(1,1),y(1,1),'.','color',colors(5,:),'markersize',20)


box on
axis([-.5 10.5 -.5 10.5])

saveas(gcf,'simulation.png')

end

function dxdt = unicycle_ode(t,x,u)

  dxdt = zeros(4,1);
  L = 0.1; d1 = 0.2;

  dxdt(1)=u(1)*cos(x(3));
  dxdt(2)=u(1)*sin(x(3));
  dxdt(3)=u(1)/L*tan(u(2));
  dxdt(4)=u(1)/d1*sin(x(3)-x(4));

end

function plot_domain

colors=get(groot,'DefaultAxesColorOrder');

% Targets
v=[9 0; 9.5  0; 9 0.5; 9.5 .5];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(2,:),'edgec',colors(2,:));

end
