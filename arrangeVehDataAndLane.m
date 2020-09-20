% Author : Viranjan Bhattacharyya
% July 2020
function [steer, velocity] = arrangeVehDataAndLane(vehList,vehUser)
    x = vehList(:,1);                              %Other vehicle data
    y = vehList(:,2);                             %x,y,v etc. are columns
    v = vehList(:,3);                              %of vehList from simultr
    laneIndex= vehList(:,4);
    %lanePos = vehList(:,5);

    xU = vehUser(1,1);                          % Current Host vehicle data
    yU = vehUser(1,2);
    vU = vehUser(1,3);
    laneIndexU = vehUser(1,4);
    %lanePosU = vehUser(1,5);
    
    v_efficient = 31;                           %define efficient vel                 
    
    % Identify Lane w.r.t. Host ( 0=>same lane ; 1=>right ; -1=>left )
    lanediff = laneIndex - laneIndexU;

    samelane = find(lanediff==0);
    right = find(lanediff==1);
    left = find(lanediff==-1);

    xsamelane = x(samelane);
    ysamelane = y(samelane);
    vsamelane = v(samelane);
    xright = x(right);
    yright = y(right);
    vright = v(right);
    xleft = x(left);
    yleft = y(left);
    vleft = v(left);
    
    % Identify front/rear
    fr = y - yU;

    front = find(fr>0);
    rear = find(fr<=0);

    xfront = x(front);
    yfront = y(front);
    vfront = v(front);
    xrear = x(rear);
    yrear = y(rear);
    vrear = v(rear);
    % Front samelane
    if length(front) > length(samelane)
        i = ismember(front,samelane);
        xfrontsamelane = xfront(i);
        yfrontsamelane = yfront(i);
        vfrontsamelane = vfront(i);
    else
        i = ismember(samelane,front);
        xfrontsamelane = xsamelane(i);
        yfrontsamelane = ysamelane(i);
        vfrontsamelane = vsamelane(i);
    end

    % Front Right
    if length(front) > length(right)
        i = ismember(front,right);
        xfrontright = xfront(i);
        yfrontright = yfront(i);
        vfrontright = vfront(i);
    else
        i = ismember(right,front);
        xfrontright = xright(i);
        yfrontright = yright(i);
        vfrontright = vright(i);
    end

    % Front Left
    if length(front) > length(left)
        i = ismember(front,left);
        xfrontleft = xfront(i);
        yfrontleft = yfront(i);
        vfrontleft = vfront(i);
    else
        i = ismember(left,front);
        xfrontleft = xleft(i);
        yfrontleft = yleft(i);
        vfrontleft = vleft(i);
    end
    % Rear Right
    if length(rear) > length(right)
        i = ismember(rear,right);
        xrearright = xrear(i);
        yrearright = yrear(i);
        vrearright = vrear(i);
    else
        i = ismember(right,rear);
        xrearright = xright(i);
        yrearright = yright(i);
        vrearright = vright(i);
    end
    % Rear Left
    if length(rear) > length(left)
        i = ismember(rear,left);
        xrearleft = xrear(i);
        yrearleft = yrear(i);
        vrearleft = vrear(i);
    else
        i = ismember(left,rear);
        xrearleft = xleft(i);
        yrearleft = yleft(i);
        vrearleft = vleft(i);
    end
    
    % Front-same lane vehicle
    dFS = yfrontsamelane-yU;
    if isempty(dFS)
        d(1) = inf;
        vel(1) = v_efficient;
    else
        d(1) = min(dFS);
        vel(1) = vfrontsamelane((dFS == d(1)));
    end
    % Front-right vehicle
    dFR = sqrt((yfrontright - yU).^2 + (xfrontright - xU).^2);
    if isempty(dFR)
        d(2)=inf;
        vel(2)=0;
    else
        d(2) = min(dFR); 
        vel(2) = vfrontright((dFR == d(2)));
    end
    % Front-left vehicle   
    dFL = sqrt((yfrontleft - yU).^2 + (xfrontleft - xU).^2);
    if isempty(dFL)
        d(3)=inf;
        vel(3)=0;
    else
        d(3) = min(dFL);
        vel(3) = vfrontleft((dFL == d(3)));
    end
    % Rear-right vehicle
    dRR = sqrt((yrearright - yU).^2 + (xrearright - xU).^2);
    if isempty(dRR)
        d(4)=inf;
        vel(4)=0;
    else
        d(4) = min(dRR);  
        vel(4) = vrearright((dRR == d(4)));
    end
    % Rear-left vehicle
    dRL = sqrt((yrearleft - yU).^2 + (xrearleft - xU).^2);
    if isempty(dRL)
        d(5)=inf;
        vel(5)=0;
    else
        d(5) = min(dRL);
        vel(5) = vrearleft((dRL == d(5)));
    end
    
    % Safety distance
    %d_safe = 5*ones(1,5) + 0.5*abs(vel - vU);
    %d_safe = 5*ones(1,5) + 0.1*(vU - vel);
    %d_safe = 10*ones(1,5);
    d_safe(1) = 30; d_safe(2) = 5 + 5*(vU - vel(2)); d_safe(3) = 5 + 5*(vU - vel(3)); d_safe(4) = 5 + 15*(vel(4) - vU); d_safe(5) = 5 + 15*(vel(5) - vU);
    
    % Lane Change
    % Middle Lane
    if (laneIndexU ~= 0 && laneIndexU ~= 2 && laneIndexU == 1)
        if vU < v_efficient
            if vel(1) >= v_efficient
                velocity = v_efficient;
                steer = 0;
            else
                if (d(3)>d_safe(3) && d(5)>d_safe(5) && d(1)<d_safe(1))
                    velocity = v_efficient;
                    steer = -1;
                elseif (d(2)>d_safe(2) && d(4)>d_safe(4) && d(1)<d_safe(1))
                    velocity = v_efficient;
                    steer = 1;
                else
                    velocity = vel(1);
                    steer = 0;
                end
            end
         else
             velocity = v_efficient;
             steer = 0;
         end
   % Right Lane  
   elseif (laneIndexU ~= 0 && laneIndexU ~= 1 && laneIndexU == 2)
       if vU < v_efficient
           if vel(1) >= v_efficient
               velocity = v_efficient;
               steer = 0;
           else
               if (d(3)>d_safe(3) && d(5)>d_safe(5) && d(1)<d_safe(1))
                   velocity = v_efficient;
                   steer = -1;
               else
                   velocity = vel(1);
                   steer = 0;
               end
           end
       else
           velocity = v_efficient;
           steer = 0;
       end
   % Left Lane
   elseif (laneIndexU ~= 2 && laneIndexU ~= 1 && laneIndexU == 0)
       if vU < v_efficient
           if vel(1) >= v_efficient
               velocity = v_efficient;
               steer = 0;
           else
               if (d(2)>d_safe(2) && d(4)>d_safe(4) && d(1)<d_safe(1))
                   velocity = v_efficient;
                   steer = 1;
               else
                   velocity = vel(1);
                   steer = 0;
               end
           end
       else
           velocity = v_efficient;
           steer = 0;
       end
   end
    
end 
 

  
