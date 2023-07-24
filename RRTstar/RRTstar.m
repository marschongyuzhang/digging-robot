clear all; close all; clc;
%%
x_I = 1; y_I = 1;          
x_G = 750; y_G = 750;      
GoalThreshold = 30;        
Delta = 30;                 
RadiusForNeib = 80;       
MaxIterations = 2500;      
UpdateTime = 50;          
DelayTime = 0.0;           
%% 
T.v(1).x = x_I;             
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;         
T.v(1).yPrev = y_I;
T.v(1).totalCost = 0;         % distance cost dd
T.v(1).indPrev = 0;         
%% 
figure(1);
ImpRgb = imread('map.png');
Imp = rgb2gray(ImpRgb);
imshow(Imp)
xL = size(Imp,1);   
yL = size(Imp,2);  
hold on
plot(x_I, y_I, 'mo', 'MarkerSize',10, 'MarkerFaceColor','m');   
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
count = 1;
pHandleList = [];
lHandleList = [];
resHandleList = [];
findPath = 0;
update_count = 0;
path.pos = [];
for iter = 1:MaxIterations
    
    %Step 1: (Sample)
    x_rand = [unifrnd(0,xL),unifrnd(0,yL)];	
    
    %Step 2: 
    minDis = sqrt((x_rand(1) - T.v(1).x)^2 + (x_rand(2) - T.v(1).y)^2);
    minIndex = 1;
    for i = 2:size(T.v,2)	
    	distance = sqrt((x_rand(1) - T.v(i).x)^2 + (x_rand(2) - T.v(i).y)^2);   
        if(distance < minDis)
            minDis = distance;
            minIndex = i;   
        end     
    end
    
    x_near(1) = T.v(minIndex).x;    
    x_near(2) = T.v(minIndex).y;
    temp_parent = minIndex;         
    temp_cost = Delta + T.v(minIndex).totalCost;   

    %Step 3: (Steer)
    theta = atan2((x_rand(2) - x_near(2)),(x_rand(1) - x_near(1)));
    x_new(1) = x_near(1) + cos(theta) * Delta;
    x_new(2) = x_near(2) + sin(theta) * Delta;  
    %plot(x_rand(1), x_rand(2), 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
    %plot(x_new(1), x_new(2), 'bo', 'MarkerSize',10,
    %'MarkerFaceColor','b'); 
    
    if ~collisionChecking(x_near,x_new,Imp) 
        continue;   
    end

    %Step 4: (NearC)
    disToNewList = [];    
    nearIndexList = [];
    for index_near = 1:count
        disTonew = sqrt((x_new(1) - T.v(index_near).x)^2 + (x_new(2) - T.v(index_near).y)^2);
        if(disTonew < RadiusForNeib)    
            disToNewList = [disToNewList disTonew];     
            nearIndexList = [nearIndexList index_near];    
        end
    end
    
    %Step 5: (ChooseParent)
    for cost_index = 1:length(nearIndexList)    
        costToNew = disToNewList(cost_index) + T.v(nearIndexList(cost_index)).totalCost;
        if(costToNew < temp_cost)    
            x_mincost(1) = T.v(nearIndexList(cost_index)).x;     
            x_mincost(2) = T.v(nearIndexList(cost_index)).y;
            if ~collisionChecking(x_mincost,x_new,Imp) 
            	continue;   
            end
        	temp_cost = costToNew;
        	temp_parent = nearIndexList(cost_index);
        end
    end
    
    %Step 6: 
    count = count+1;    
    
    T.v(count).x = x_new(1);          
    T.v(count).y = x_new(2); 
    T.v(count).xPrev = T.v(temp_parent).x;     
    T.v(count).yPrev = T.v(temp_parent).y;
    T.v(count).totalCost = temp_cost; 
    T.v(count).indPrev = temp_parent;     
    
   l_handle = plot([T.v(count).xPrev, x_new(1)], [T.v(count).yPrev, x_new(2)], 'b', 'Linewidth', 2);
   p_handle = plot(x_new(1), x_new(2), 'ko', 'MarkerSize', 4, 'MarkerFaceColor','k');
   
   pHandleList = [pHandleList p_handle];    
   lHandleList = [lHandleList l_handle];
   pause(DelayTime);
    %Step 7: (rewire)
    for rewire_index = 1:length(nearIndexList)
        if(nearIndexList(rewire_index) ~= temp_parent)    
            newCost = temp_cost + disToNewList(rewire_index);    
            if(newCost < T.v(nearIndexList(rewire_index)).totalCost)    
                x_neib(1) = T.v(nearIndexList(rewire_index)).x;     
                x_neib(2) = T.v(nearIndexList(rewire_index)).y;
                if ~collisionChecking(x_neib,x_new,Imp) 
                    continue;   
                end
                T.v(nearIndexList(rewire_index)).xPrev = x_new(1);      
                T.v(nearIndexList(rewire_index)).yPrev = x_new(2);
                T.v(nearIndexList(rewire_index)).totalCost = newCost;
                T.v(nearIndexList(rewire_index)).indPrev = count;       
                
                %delete(pHandleList());
                %delete(lHandleList(nearIndexList(rewire_index)));
                lHandleList(nearIndexList(rewire_index)) = plot([T.v(nearIndexList(rewire_index)).x, x_new(1)], [T.v(nearIndexList(rewire_index)).y, x_new(2)], 'r', 'Linewidth', 2);

                %pHandleList = [pHandleList p_handle];    
                %lHandleList = [lHandleList l_handle];
            end
        end
    end
    
    %Step 8:
    disToGoal = sqrt((x_new(1) - x_G)^2 + (x_new(2) - y_G)^2);
    if(disToGoal < GoalThreshold && ~findPath)    
        findPath = 1;

        count = count+1;    
        Goal_index = count;
        T.v(count).x = x_G;          
        T.v(count).y = y_G; 
        T.v(count).xPrev = x_new(1);     
        T.v(count).yPrev = x_new(2);
        T.v(count).totalCost = T.v(count - 1).totalCost + disToGoal;
        T.v(count).indPrev = count - 1;     
    end
    
    if(findPath == 1)
        update_count = update_count + 1;
        if(update_count == UpdateTime)
            update_count = 0;
            j = 2;
            path.pos(1).x = x_G; 
            path.pos(1).y = y_G;
            pathIndex = T.v(Goal_index).indPrev;
            while 1     
                path.pos(j).x = T.v(pathIndex).x;
                path.pos(j).y = T.v(pathIndex).y;
                pathIndex = T.v(pathIndex).indPrev;    
                if pathIndex == 0
                    break
                end
                j=j+1;
            end  
            
            for delete_index = 1:length(resHandleList)
            	delete(resHandleList(delete_index));
            end
            for j = 2:length(path.pos)
                res_handle = plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'g', 'Linewidth', 4);
                resHandleList = [resHandleList res_handle];
            end
        end
    end  
	pause(DelayTime); 
end

for delete_index = 1:length(resHandleList)
	delete(resHandleList(delete_index));
end
for j = 2:length(path.pos)
	res_handle = plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'g', 'Linewidth', 4);
	resHandleList = [resHandleList res_handle];
end
            
disp('The path is found!');

