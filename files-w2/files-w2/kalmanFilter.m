function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
     C=[1 0 0 0;...
         0 1 0 0];
     del_t=t-previous_t;
     A=[1 0 del_t 0;...
         0 1 0 del_t;...
         0 0 1 0;...
         0 0 0 1];
    Q= [del_t*del_t 0    del_t 0 ;...
               0    del_t*del_t  0  del_t;...
               del_t     0    1   0;...
               0     del_t     0  1];
     R=[0.0080 0;...
         0 0.0080];

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    %vx = (x - state(1)) / (t - previous_t);
    %vy = (y - state(2)) / (t - previous_t);
    % Predict 330ms into the future
    %predictx = x + vx * 0.330;
    %predicty = y + vy * 0.330;
    % State is a four dimensional element
    P=A*param.P*A'+Q;
    K=(P*C')*pinv(C*P*C'+R);
    state_=A*state'+K*([x;y]-C*A*state');
    param.P=P-K*C*P;
    predictx=state(1)+state(3)*0.330;
    predicty=state(2)+state(4)*0.330;
    state=state_';
end
