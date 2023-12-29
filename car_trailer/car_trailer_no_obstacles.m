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
% initial state
x0=[0 10 0 0];



controller=SymbolicSet('car_trailer_controller.bdd','projection',[1 2 3 4]);
target=SymbolicSet('car_trailer_target.bdd');

% ADDED by Mohammed (line 39-43)
domain_points = controller.points;
plot(domain_points(:,1),domain_points(:,2),'.')
% domain_points(find(domain_points(:,1)==6.8, 1, 'first'),:)
% sample one random row from feasible region and set as initial state
y=[domain_points(randperm(length(domain_points),1),:)];

% get a leftmost point in the domain
% [~,I] = min(domain_points(:,1));
% x0=domain_points(I,:)
y=x0;% - originally
v=[];

while(1)

  
  if (target.isElement(y(end,:))) 
    break;
  end 

  u=controller.getInputs(y(end,:));
  v=[v; u(1,:)];
  [t x]=ode45(@unicycle_ode,[0 .3], y(end,:), [], u(1,:));

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
  L = 0.25; d1 = 0.4;

  dxdt(1)=u(1)*cos(x(3));
  dxdt(2)=u(1)*sin(x(3));
  dxdt(3)=u(1)/L*tan(u(2));
  dxdt(4)=u(1)/d1*sin(x(3)-x(4));

end

function plot_domain

colors=get(groot,'DefaultAxesColorOrder');

v=[9 0; 9.5  0; 9 0.5; 9.5 .5];
patch('vertices',v,'faces',[1 2 4 3],'facec',colors(2,:),'edgec',colors(2,:));


% v=[1     0  ;1.2  0   ; 1     9    ; 1.2 9   ];
% patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
% v=[2.2   0  ;2.4  0   ; 2.2   5    ; 2.4 5   ];
% patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
% v=[2.2   6  ;2.4  6   ; 2.2   10   ; 2.4 10  ];
% patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
% v=[3.4   0  ;3.6  0   ; 3.4   9    ; 3.6 9   ];
% patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
% v=[4.6   1  ;4.8  1   ; 4.6   10   ; 4.8 10  ];
% patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
% v=[5.8   0  ;6    0   ; 5.8   6    ; 6   6   ];
% patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
% v=[5.8   7  ;6    7   ; 5.8   10   ; 6   10  ];
% patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
% v=[7     1  ;7.2  1   ; 7     10   ; 7.2 10  ];
% patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
% v=[8.2   0  ;8.4  0   ; 8.2   8.5  ; 8.4 8.5 ];
% patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
% v=[8.4   8.3;9.3  8.3 ; 8.4   8.5  ; 9.3 8.5 ];
% patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
% v=[9.3   7.1;10   7.1 ; 9.3   7.3  ; 10  7.3 ];
% patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
% v=[8.4   5.9;9.3  5.9 ; 8.4   6.1  ; 9.3 6.1 ];
% patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
% v=[9.3   4.7;10   4.7 ; 9.3   4.9  ; 10  4.9 ];
% patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
% v=[8.4   3.5;9.3  3.5 ; 8.4   3.7  ; 9.3 3.7 ];
% patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));
% v=[9.3   2.3;10   2.3 ; 9.3   2.5  ; 10  2.5 ];
% patch('vertices',v,'faces',[1 2 4 3],'facec',colors(1,:),'edgec',colors(1,:));


end
