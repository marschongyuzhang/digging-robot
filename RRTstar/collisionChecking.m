function feasible=collisionChecking(startPose,goalPose,map)

feasible=true;
dir = atan2(goalPose(2)-startPose(2),goalPose(1)-startPose(1));
for r = 0:0.5:sqrt(sum((startPose-goalPose).^2))      
    %posCheck = startPose + r.*[sin(dir) cos(dir)];      
    posCheck = startPose + r.*[cos(dir) sin(dir)];      
    
    if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) ...
        && feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) ...
        && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible = false;
        break;
    end
    if ~feasiblePoint([floor(goalPose(1)),ceil(goalPose(2))],map)
        feasible = false; 
    end
    
end

function feasible = feasiblePoint(point,map)
feasible = true;
if ~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 ...
    && point(2)<=size(map,2) && map(point(2),point(1))==255)
    feasible = false;   % falls ja
end