%
% diff_drive.m
%
% created on: 22.11.2023
%     author: mohammed adib oumer
%
% see readme file for more information on the diff_drive example
%
% you need to run ./diff_drive binary first 
%
% so that the files: diff_drive_ss.bdd 
%                    diff_drive_obst.bdd
%                    diff_drive_target.bdd
%                    diff_drive_controller.bdd 
% are created
%

function diff_drive
clear set
close all

%% simulation

% target set
lb=[9 0];
ub=lb+0.5;
v=[9 0; 9.5  0; 9 0.5; 9.5 .5];
% initial state
x0= [.4,.4,0];


controller=SymbolicSet('diff_drive_controller.bdd','projection',[1 2 3]);
target=SymbolicSet('diff_drive_target.bdd');

domain_points = controller.points;
plot(domain_points(:,1),domain_points(:,2),'.');

% sample one random row from feasible region and set as initial state
% y=[domain_points(randperm(length(domain_points),1),:)];

% get a leftmost point in the domain
% [~,I] = min(domain_points(:,1));
% x0=domain_points(I,:);
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

%% plot the diff_drive domain
% colors
colors=get(groot,'DefaultAxesColorOrder');


% load the symbolic set containing the abstract state space
set=SymbolicSet('diff_drive_ss.bdd','projection',[1 2]);
plotCells(set,'facecolor','none','edgec',[0.8 0.8 0.8],'linew',.1)
hold on

% load the symbolic set containig obstacles
set=SymbolicSet('diff_drive_obst.bdd','projection',[1 2]);
plotCells(set,'facecolor',colors(1,:)*0.5+0.5,'edgec',colors(1,:),'linew',.1)

% plot the real obstacles and target set
plot_domain

% load the symbolic set containig target set
set=SymbolicSet('diff_drive_target.bdd','projection',[1 2]);
plotCells(set,'facecolor',colors(2,:)*0.5+0.5,'edgec',colors(2,:),'linew',.1)

% plot initial state  and trajectory for first controller
plot(y(:,1),y(:,2),'k.-')
plot(y(1,1),y(1,1),'.','color',colors(5,:),'markersize',20)

box on
axis([-.5 10.5 -.5 10.5])

saveas(gcf,'simulation.png')

end

function dxdt = ode_equations(t,x,u)

  dxdt = zeros(3,1);
  rad = 0.06; L = 0.4;

  dxdt(1)=(rad/2.0)*(u(2)+u(1))*cos(x(3));
  dxdt(2)=(rad/2.0)*(u(2)+u(1))*sin(x(3));
  dxdt(3)=(rad/L)*(u(2)-u(1));

end

function plot_domain

colors=get(groot,'DefaultAxesColorOrder');

% Targets
v=[9 0; 9.5  0; 9 0.5; 9.5 .5];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(2,:),'edgec',colors(2,:));

% Obstacles
v=[1     0  ;1.2  0   ; 1     9    ; 1.2 9   ]; 
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
v=[2.2   0  ;2.4  0   ; 2.2   5    ; 2.4 5   ];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
v=[2.2   6  ;2.4  6   ; 2.2   10   ; 2.4 10  ];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
v=[3.4   0  ;3.6  0   ; 3.4   9    ; 3.6 9   ];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
v=[4.6   1  ;4.8  1   ; 4.6   10   ; 4.8 10  ];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
v=[5.8   0  ;6    0   ; 5.8   6    ; 6   6   ];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
v=[5.8   7  ;6    7   ; 5.8   10   ; 6   10  ];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
v=[7     1  ;7.2  1   ; 7     10   ; 7.2 10  ];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
v=[8.2   0  ;8.4  0   ; 8.2   8.5  ; 8.4 8.5 ];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
v=[8.4   8.3;9.3  8.3 ; 8.4   8.5  ; 9.3 8.5 ];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
v=[9.3   7.1;10   7.1 ; 9.3   7.3  ; 10  7.3 ];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
v=[8.4   5.9;9.3  5.9 ; 8.4   6.1  ; 9.3 6.1 ];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
v=[9.3   4.7;10   4.7 ; 9.3   4.9  ; 10  4.9 ];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
v=[8.4   3.5;9.3  3.5 ; 8.4   3.7  ; 9.3 3.7 ];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
v=[9.3   2.3;10   2.3 ; 9.3   2.5  ; 10  2.5 ];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));


end
