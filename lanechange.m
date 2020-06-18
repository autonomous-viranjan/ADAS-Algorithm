function [steer, velocity] = lanechange(vel,d,d_safe,v_efficient,laneIndexU)    
    if (laneIndexU == 1) && (vel(1)>=v_efficient)
        velocity = v_efficient;
        steer = 0;

    elseif (laneIndexU == 1) && (vel(3)>v_efficient) && (vel(5)<vel(3)) && (d(3)>d_safe(3)) && (d(5)>d_safe(5))
        velocity = vel(3);
        steer = -1;    
    elseif (laneIndexU == 1) && (vel(3)==0) && (vel(5)<v_efficient) && (d(5)>d_safe(5))
        velocity = v_efficient;
        steer = -1;
    elseif (laneIndexU == 1) && (vel(2)>v_efficient) && (vel(4)<vel(2)) && (d(2)>d_safe(2)) && (d(4)>d_safe(4))
        velocity = vel(2);
        steer = 1;     
    elseif (laneIndexU == 1) && (vel(2)==0) && vel(4)<v_efficient && (d(4)>d_safe(4))
        velocity = v_efficient;
        steer = 1;
    
    %In Left lane
    %check only 1,2,4
    elseif (laneIndexU == 0) && (vel(1)>=v_efficient)
        velocity = v_efficient;
        steer = 0;

    elseif (laneIndexU == 0) && (vel(2)>=v_efficient) && (vel(4)<vel(2)) && (d(2)>=d_safe(2)) && (d(4)>d_safe(4))
        velocity = vel(2);
        steer = 1;      

    elseif (laneIndexU == 0) && (vel(2)==0) && (vel(4)<v_efficient) && (d(4)>d_safe(4))
        velocity = v_efficient;
        steer = 1;
    
    %In Right lane
    %check only 1,3,5
    elseif (laneIndexU == 2) && (vel(1)>=v_efficient)
        velocity = v_efficient;
        steer = 0;

    elseif (laneIndexU == 2) && (vel(3)>=v_efficient) && (vel(5)<vel(3)) && (d(3)>d_safe(3)) && (d(5)>d_safe(5))
        velocity = vel(3);
        steer = -1;    
    
    elseif (laneIndexU == 2) && (vel(3)==0) && (vel(5)<v_efficient) && (d(5)>d_safe(5))
        velocity = v_efficient;
        steer = -1;
    else
        velocity = vel(1);
        steer = 0;
    end       
             
end 