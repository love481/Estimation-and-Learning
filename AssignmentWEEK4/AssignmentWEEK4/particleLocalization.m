% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
 myResolution = param.resol;
% % the origin of the map in pixels
 myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.



% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M =400;                            % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
%P = repmat(myPose(:,1), [1, M]);
weight=ones(1,M);
correction=ones(1,M);
weight=weight/sum(weight);
predPart=repmat(myPose(:,1), [1, M]);
 for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
% 
%     % 1) Propagate the particles 
%

predPart=predPart+random('norm',0,0.1,3,M);

%       
%     % 2) Measurement Update 
%     %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
%
for i=1:M
    lidar_local = [(ranges(:,j).*cos(scanAngles+predPart(3,i)))+predPart(1,i) ...
    (-ranges(:,j).*sin(scanAngles+predPart(3,i)))+predPart(2,i)];
lidar_local=ceil(myResolution*lidar_local)+myOrigin';
if (sum(lidar_local(:,1)>size(map,2))||sum(lidar_local(:,1)<=0)||sum(lidar_local(:,2)>size(map,1))||sum(lidar_local(:,2)<=0))
    correction(i)=0;
else
    index=sub2ind(size(map),lidar_local(:,2),lidar_local(:,1));
    correction(i)=sum(map(index)>0.49);%lidar and map bot occupied
    correction(i)=correction(i)-sum(map(index)<0.49)*0.5; % lidar occupy and map free
    if correction(i)<0
        correction(i)=0;
    end
    %correction(i)=correction(i) -sum(map(setdiff(1:numel(map),index))>0.49)*5;% lidar free and map occupy
     %correction(i)=correction(i)+sum(map(setdiff(1:numel(map),index))<0.49)*10;% lidar free and map free
end
end
 weight=weight.*correction;
 weight=weight/sum(weight);
 [~,ind]=max(weight);
 myPose(:,j)=predPart(:,ind);
 Neff=1/sum(weight.^2);
if (Neff<200)
 % this is performing latin hypercube sampling on wk
      edges = min([0 cumsum(weight')'],1); % protect against accumulated round-off
      edges(end) = 1;                 % get the upper edge exact
      u1 = rand/M;
      % this works like the inverse of the empirical distribution and returns
      % the interval where the sample is to be found
      [~, idx] = histc(u1:1/M:1, edges);
predPart = predPart(:,idx);                    % extract new particles
weight = repmat(1/M, 1, M);   
%     %   2-2) For each particle, calculate the correlation scores of the particles
%
%     %   2-3) Update the particle weights         
%  
%     %   2-4) Choose the best particle to update the pose
%     
%     % 3) Resample if the effective number of particles is smaller than a threshold
% 
%     % 4) Visualize the pose on the map as needed
%    
% 
end
 end
 figure;
imagesc(map); hold on;
colormap('gray');
axis equal;
hold on;
plot(myPose(1,:)*param.resol+param.origin(1), ...
    myPose(2,:)*param.resol+param.origin(2), 'r.-');
end

