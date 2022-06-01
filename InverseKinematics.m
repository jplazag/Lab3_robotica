
function thetas = InverseKinematics(Robot,l,Pose)

    
% %     q = deg2rad([120 98 -90 10]);
%     q = deg2rad([0 0 0 0]);
% 
%     Robot.plot(q,'notiles');
%     Robot.teach();
%     hold on;
%     trplot(eye(4),'rgb','arrow','length',25,'frame','or')
%     
%     M = eye(4);
%     for i=1:Robot.n
%         M = M * L(i).A(q(i));
%         trplot(M,'rgb','arrow','frame',num2str(i),'length',15 + 3*(-1)^i)
%     end
    
    
    
    HT0 = Pose; %Robot.fkine(q);
    
    d_T = HT0(1:3,4);
    approach = HT0(1:3,3);
    W = d_T - l(4)*approach;
    
    x_p = HT0(1,4);
    y_p = HT0(2,4);
    
    th1 = atan2(y_p,x_p);

    if abs(th1)>pi/2
        th1_m = atan2(-y_p,-x_p);
    else
        th1_m = th1;
    end
    x_trian = sqrt( W(1)^2 + W(2)^2);
    y_trian = W(3) - l(1);
    
    cos_th3 = (x_trian^2 + y_trian^2 - l(2)^2 - l(3)^2)/(2*l(2)*l(3));
    
    sin_th3 = sqrt(1-cos_th3^2); %Codo arriba (Es contrario a lo visto en clase porque acá el movto positivo es en el sentido del reloj)
    
    th3_up = atan2(real(sin_th3),cos_th3);
    
    th3_do = -th3_up; %Codo abajo
    
    alpha = atan2(-y_trian,x_trian); %Acá y se toma negativa puesto que el mvto de las juntas es contrario al sentido de la convención 
    beta= atan2(-l(3)*sin(abs(th3_do)),l(2)+l(3)*cos(abs(th3_do)));
    

    th2_do = alpha - beta; %Codo abajo
    th2_do = (pi/2+th2_do);
    
    th2_up = alpha + beta; %Codo arriba
    th2_up = (pi/2+th2_up);
    
    
    
    H30_do = Robot.A([1 2 3],[th1_m th2_do th3_do]);
    
    HT3_do = H30_do^(-1)*HT0;
    
    th4_do = atan2(-HT3_do(1,1),HT3_do(2,1));


    H30_up = Robot.A([1 2 3],[th1_m th2_up th3_up]);

    HT3_up = H30_up^(-1)*HT0;
    
    th4_up = atan2(-HT3_up(1,1),HT3_up(2,1));
    
    thetas = [th1 th2_do th3_do th4_do; ...
              th1 th2_up th3_up th4_up]; 
%     thetas = rad2deg([th1 th2_do th3_do th4_do; ...
%                       th1 th2_up th3_up th4_up])

    

end

