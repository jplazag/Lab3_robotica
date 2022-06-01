function movePX(msg,cliente,thetas,mov_gripper)

    if mov_gripper
        msg.AddrName = "Goal_Position";
        msg.Id = 5;
        msg.Value = mapfun(rad2deg(thetas),-150,150,0,1023);
        call(cliente,msg);
    else
        msg.AddrName = "Goal_Position";
        pause(0.1);
        q = rad2deg(thetas(2,:));
    %     q(5) = o_gripper;
    for i=4:-1:1
        msg.Id = i;
        msg.Value = mapfun(q(i),-150,150,0,1023);
        call(cliente,msg);
    end

    end
    

end