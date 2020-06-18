function [steer, velocity] = arrangeVehDataAndLane(vehList,vehUser)
    x = vehList(:,1);                              %Other vehicle data
    %y = vehList(:,2);                             %x,y,v etc. are columns
    v = vehList(:,3);                              %of vehList from simultr
    laneIndex= vehList(:,4);
    lanePos = vehList(:,5);

    xU = vehUser(1,1);                          % Current Host vehicle data
    %yU = vehUser(1,2);
    vU = vehUser(1,3);
    laneIndexU = vehUser(1,4);
    lanePosU = vehUser(1,5);
    
    v_efficient = 55;                           %define efficient vel                 
    
    % Identify Lane w.r.t. Host ( 0=>same lane ; 1=>right ; -1=>left )
    lanediff = laneIndex - laneIndexU;

    samelane = find(lanediff==0);
    right = find(lanediff==1);
    left = find(lanediff==-1);

    xsamelane = x(samelane);
    lanePossamelane = lanePos(samelane);
    vsamelane = v(samelane);
    xright = x(right);
    lanePosright = lanePos(right);
    vright = v(right);
    xleft = x(left);
    lanePosleft = lanePos(left);
    vleft = v(left);
    
    % Identify front/rear
    fr = lanePos - lanePosU;

    front = find(fr>0);
    rear = find(fr<=0);

    xfront = x(front);
    lanePosfront = lanePos(front);
    vfront = v(front);
    xrear = x(rear);
    lanePosrear = lanePos(rear);
    vrear = v(rear);
    % Front samelane
    if length(front) > length(samelane)
        i = ismember(front,samelane);
        xfrontsamelane = xfront(i);
        lanePosfrontsamelane = lanePosfront(i);
        vfrontsamelane = vfront(i);
    else
        i = ismember(samelane,front);
        xfrontsamelane = xsamelane(i);
        lanePosfrontsamelane = lanePossamelane(i);
        vfrontsamelane = vsamelane(i);
    end

    % Front Right
    if length(front) > length(right)
        i = ismember(front,right);
        xfrontright = xfront(i);
        lanePosfrontright = lanePosfront(i);
        vfrontright = vfront(i);
    else
        i = ismember(right,front);
        xfrontright = xright(i);
        lanePosfrontright = lanePosright(i);
        vfrontright = vright(i);
    end

    % Front Left
    if length(front) > length(left)
        i = ismember(front,left);
        xfrontleft = xfront(i);
        lanePosfrontleft = lanePosfront(i);
        vfrontleft = vfront(i);
    else
        i = ismember(left,front);
        xfrontleft = xleft(i);
        lanePosfrontleft = lanePosleft(i);
        vfrontleft = vleft(i);
    end
    % Rear Right
    if length(rear) > length(right)
        i = ismember(rear,right);
        xrearright = xrear(i);
        lanePosrearright = lanePosrear(i);
        vrearright = vrear(i);
    else
        i = ismember(right,rear);
        xrearright = xright(i);
        lanePosrearright = lanePosright(i);
        vrearright = vright(i);
    end
    % Rear Left
    if length(rear) > length(left)
        i = ismember(rear,left);
        xrearleft = xrear(i);
        lanePosrearleft = lanePosrear(i);
        vrearleft = vrear(i);
    else
        i = ismember(left,rear);
        xrearleft = xleft(i);
        lanePosrearleft = lanePosleft(i);
        vrearleft = vleft(i);
    end
    
    % Front-same lane vehicle
    dFS = lanePosfrontsamelane-lanePosU;
    if isempty(dFS)
        d(1) = inf;
        vel(1) = v_efficient;
    else
        d(1) = min(dFS);
        vel(1) = vfrontsamelane((dFS == d(1)));
    end
    % Front-right vehicle
    dFR = sqrt((lanePosfrontright - lanePosU).^2 + (xfrontright - xU).^2);
    if isempty(dFR)
        d(2)=inf;
        vel(2)=0;
    else
        d(2) = min(dFR); 
        vel(2) = vfrontright((dFR == d(2)));
    end
    % Front-left vehicle   
    dFL = sqrt((lanePosfrontleft - lanePosU).^2 + (xfrontleft - xU).^2);
    if isempty(dFL)
        d(3)=inf;
        vel(3)=0;
    else
        d(3) = min(dFL);
        vel(3) = vfrontleft((dFL == d(3)));
    end
    % Rear-right vehicle
    dRR = sqrt((lanePosrearright - lanePosU).^2 + (xrearright - xU).^2);
    if isempty(dRR)
        d(4)=inf;
        vel(4)=0;
    else
        d(4) = min(dRR);  
        vel(4) = vrearright((dRR == d(4)));
    end
    % Rear-left vehicle
    dRL = sqrt((lanePosrearleft - lanePosU).^2 + (xrearleft - xU).^2);
    if isempty(dRL)
        d(5)=inf;
        vel(5)=0;
    else
        d(5) = min(dRL);
        vel(5) = vrearleft((dRL == d(5)));
    end
    
    % Safety distance
    d_safe = 5*ones(1,5) + 0.1*abs(vel - vU);
    
        
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
    
end
  