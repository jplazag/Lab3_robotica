rosinit;
%%
clear;
% clf("reset")

cliente = rossvcclient('/dynamixel_workbench/dynamixel_command'); %Creación de cliente de pose y posición
msg = rosmessage(cliente); %Creación de mensaje

msg.AddrName = "Torque_Limit";

values = [800 800 400 400];

for i=1:4
        msg.Id = i+5;
        msg.Value = values(i);
        call(cliente,msg);
%         pause(0.5);
end


l = [3.9 10.57 10.6 6.57]; % Longitudes eslabones
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




move = 0;
T0 =  Robot.fkine([1.5544 0.7056 1.9226 0.3988]); %Robot.fkine([0 0 0 0]);
n = 2; %Number of poses
d = 1;

should_move = false;
move_with_T = false;

while(true)

    movements = ["trax" "tray" "traz" "rot"];

    str = sprintf('ACTUAL MOVEMENT: %s \n',movements(mod(move,4)+1));

    prompt = "What do you want to do? \n \n W: Next movement type \n " + ...
        "S: Previous movement type \n D: Move in positive direction  \n " + ...
        "A: Move in negative direction \n \n" + str;

    if move_with_T
        prompt = prompt + "  respect the tool";
    else
        prompt = prompt + "  respect the base";
    end
    
    x = input(prompt,'s');
    
    switch lower(x)
        case 'w'
            move = move + 1;
        case 's'
            if move > 0
                move = move - 1;
            end
        case 'a'
            if move_with_T
                switch mod(move,4)
                    case 0            
                        T1 = T0*transl(-d,0,0);
                    case 1
                        T1 = T0*transl(0,-d,0);
                    case 2
                        T1 = T0*transl(0,0,-d);
                    case 3
                        T1 = T0*troty(-pi/8);                    
                end
            else
                switch mod(move,4)
                    case 0            
                        T1 = transl(-d,0,0)*T0;
                    case 1
                        T1 = transl(0,-d,0)*T0;
                    case 2
                        T1 = transl(0,0,-d)*T0;
                    case 3
                        T1 = T0*troty(-pi/8);                    
                end
            end
            should_move = true;

            if abs(T1(1:3,4)-[0 0 l(1)])> sum(l(2:4))
                should_move = false;
                T1 = T0;
                disp("Out of the workspace")
                pause(1);
            end
        case 'd'
            if move_with_T
                switch mod(move,4)
                    case 0            
                        T1 = T0*transl(d,0,0);
                    case 1
                        T1 = T0*transl(0,d,0);
                    case 2
                        T1 = T0*transl(0,0,d);
                    case 3
                        T1 = T0*troty(pi/8);
                end
            else
                switch mod(move,4)
                    case 0            
                        T1 = transl(d,0,0)*T0;
                    case 1
                        T1 = transl(0,d,0)*T0;
                    case 2
                        T1 = transl(0,0,d)*T0;
                    case 3
                        T1 = T0*troty(pi/8);
                end
            end
            should_move = true;

            if abs(T1(1:3,4)-[0 0 l(1)])> sum(l(2:4))
                should_move = false;
                T1 = T0;
                disp("Out of the workspace");
                pause(1);
            end

        case 'c'
            move_with_T = true;
    end
    
    
    if should_move
        % ctraj
        T01 = ctraj(T0,T1,n);
        % ciclo para calcular y graficar el robot
        
        for i=1:n
           thetas = InverseKinematics(Robot,l,T01(:,:,i));
           Robot.plot(thetas(2,:),'notiles','noname')
%            hold on;
%            Robot.teach()
           hold on;
           trplot(eye(4),'rgb','arrow','length',25,'frame','or')
           hold on
           plot3(T01(1,4,i),T01(2,4,i),T01(3,4,i),'ro')
           hold on;
           movePX(msg,cliente,thetas,false)
        end
        
    
    
        T0 = T1;
    end
    
%     press_to_pass = input("");
    should_move = false;
    clc;
end




