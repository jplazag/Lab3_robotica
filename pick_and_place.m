rosinit;
%%
clear;
clf("reset")

cliente = rossvcclient('/dynamixel_workbench/dynamixel_command'); %Creación de cliente de pose y posición
% msg = rosmessage(cliente); %Creación de mensaje
% 
% msg.AddrName = "Torque_Limit";
msg = rosmessage(cliente); %Creación de mensaje




l = [13.27 10.3 10.3 6.57]; % Longitudes eslabones

l_T = 5.5;
% Definicion del robot RTB
L(1) = Link('revolute','alpha',-pi/2,'a',0,      'd',l(1),'offset',0);
L(2) = Link('revolute','alpha',0,    'a',l(2),   'd',0,   'offset',-pi/2,'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,    'a',l(3),   'd',0,   'offset',0,    'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,    'a',l(4),   'd',0,   'offset',0);

Robot = SerialLink(L,'name','Px');

Robot.tool = [0 0 1 0;
              1 0 0 0;
              0 1 0 0;
              0 0 0 1];


T0 =   Robot.fkine([0 0 0 0]);
n = 20; %Number of poses

% T1 = Robot.fkine([1.5544 0.7056 1.9226 0.3988]);

% T2 = 
% 
% T3 =
% 
% T4 =


T1 = transl(2.5,-10,-sum(l) + l_T - 1)*T0*troty(pi); %Left
Tt1 = transl(0,0,sum(l)/2 - l_T)*T1;

TFt = transl(14.5,-0.8,-sum(l) + 2*l_T - 1)*T0*troty(pi);

T2 = transl(14.5,-0.8,-sum(l) + l_T)*T0*troty(pi); %Front
Tt2 = transl(0,0,sum(l)/2 - l_T)*T2;


T3 = transl(3.5,10,-sum(l) + l_T - 1)*T0*troty(pi);%Right
Tt3 = transl(0,0,sum(l)/2 - l_T)*T3;

T4 = transl(14.5,-0.8,-sum(l) + l_T + 1)*T0*troty(pi); %Front
Tt4 = Tt2;


Mov = cat(3,  ctraj(T0,Tt1,n), ... 
              ctraj(Tt1,T1,n), ... %Take the base
              ctraj(T1,Tt1,n), ...
              ctraj(Tt1,Tt2,n), ...
              ctraj(Tt2,TFt,n), ...
              ctraj(TFt,T2,n), ... %Put the base
              ctraj(T2,Tt2,n), ...
              ctraj(Tt2,Tt3,n), ...
              ctraj(Tt3,T3,n), ... %Take the disk
              ctraj(T3,Tt3,n), ...
              ctraj(Tt3,Tt4,n), ...
              ctraj(Tt4,TFt,n), ... 
              ctraj(TFt,T4,n), ... %Put the disk
              ctraj(T4,Tt4,n));

o_gripper = [ 0 -0.9408 -0.9408 -0.9408 -0.9408 ...
              0 0 0 -0.9408 -0.9408 -0.9408 -0.9408 0 0];

view(70,20)
for i=1:14*n
   thetas = InverseKinematics(Robot,l,Mov(:,:,i));
   movePX(msg,cliente,thetas, false);
   Robot.plot(thetas(2,:),'notiles','noname')
   hold on;
   trplot(eye(4),'rgb','arrow','length',25,'frame','or')
   hold on
   plot3(Mov(1,4,i),Mov(2,4,i),Mov(3,4,i),'ro')
   hold on;

   if mod(i,n) == 0
       movePX(msg,cliente,o_gripper(i/n), true);
       pause(1);
   end

end

