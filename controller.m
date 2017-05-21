function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

% initialize parameters
m= params.mass;
g= params.gravity;

e= s_des(1)-s(1);
e_dot= s_des(2)-s(2);

k_p= 31; 
k_v= 7;

%u=0;

z_2dot_des= 0;


u = m*(z_2dot_des + k_p*e + k_v*e_dot + g)


end

