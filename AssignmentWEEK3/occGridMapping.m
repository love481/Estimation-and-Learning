% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
%Parameters 

% the number of grids for 1 meter.
 myResol = param.resol;
% % the initial map size in pixels
 myMap = zeros(param.size);
% % the origin of the map in pixels
 myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
 lo_occ = param.lo_occ;
 lo_free = param.lo_free; 
 lo_max = param.lo_max;
 lo_min = param.lo_min;

 N = size(pose,2);
 for j = 1:N % for each time,
% 
%       
%     % Find grids hit by the rays (in the gird map coordinate)
% 
lidar_local = [(ranges(:,j).*cos(scanAngles+pose(3,j)))+pose(1,j) ...
    (-ranges(:,j).*sin(scanAngles+pose(3,j)))+pose(2,j)];
lidar_local=ceil(myResol*lidar_local)+myorigin';

% 
%     % Find occupied-measurement cells and free-measurement cells
%   
robotpos=ceil(myResol*pose(1:2,j))+myorigin;
for i=1:size(lidar_local,1)
 [freex, freey] = bresenham(robotpos(1),robotpos(2),lidar_local(i,1),lidar_local(i,2));
  myMap(lidar_local(i,2),lidar_local(i,1))=myMap(lidar_local(i,2),lidar_local(i,1))+lo_occ;
 free = sub2ind(size(myMap),freey,freex);
 myMap(free)=myMap(free)-lo_free;
end
myMap(myMap>lo_max)=lo_max;
myMap(myMap<lo_min)=lo_min;
% 
%     % Update the log-odds
%   
% 
%     % Saturate the log-odd values
%     
% 
%     % Visualize the map as needed
%    
% 
 end

end

