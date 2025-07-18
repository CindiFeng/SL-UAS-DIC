syms x(t) y(t) z(t) alpha(t) beta(t) phi(t) theta(t) psi(t) t
syms m_q m_p Ixx Iyy Izz l_p g real
syms A B C 
syms 'F_l%d' [3 1]

% %% Sub in params
% m_q = params.uav_mass;
% m_p = params.pld_mass;
% l_p = params.cable_len;
% g = sim_.g;

%% Kinematics
% R_IB = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
%         cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
%                 -sin(theta)                            sin(phi)*cos(theta)                            cos(phi)*cos(theta)];
% 
% S_B =[1            0             -sin(theta);
%       0      cos(phi)    cos(theta)*sin(phi);
%       0     -sin(phi)    cos(phi)*cos(theta)];

R_IP = [ cos(alpha)     sin(alpha)*sin(beta)       sin(alpha)*cos(beta);
                 0                 cos(beta)                 -sin(beta);
        -sin(alpha)     cos(alpha)*sin(beta)       cos(alpha)*cos(beta)];

x_q = [x;y;z]; % quad pos
v_q = diff(x_q,t);
vdot_q = diff(v_q,t);

x_p = x_q - R_IP*[0;0;l_p]; % pld pos
v_p = diff(x_p,t);
vdot_p = diff(v_p,t);

%% Euler-Lagrange

T_trans = 0.5*m_p*(v_p.')*v_p + 0.5*m_q*(v_q.')*v_q; % kinetic energy

% Note: quad rotational energy excluded as quad attitude is indepedent of
%       payload dynamics and the translational system
T_rot = 0.5*m_p*l_p^2*(diff(alpha,t)^2 + diff(beta,t)^2); 

V_ = m_p*g*[0 0 1]*x_p + m_q*g*[0 0 1]*x_q; % potential energy

L_ = (T_trans + T_rot) - V_; % Lagrangian

q_coord = [x_q.', alpha, beta]; 
qdot_coord = diff(q_coord,t);
q_coord = formula(q_coord); % convert symfun to sym
qdot_coord = formula(qdot_coord);

F_vec = [F_l.',0,0];

% Solve Euler-Lagrange eqn
for i = 1:length(q_coord)
    eqn = diff(diff(L_,qdot_coord(i)),t) - diff(L_,q_coord(i));
    soln(i) = isolate(eqn==F_vec(i), diff(qdot_coord(i),t)); % assemble EOM
end

soln_rhs = simplify(rhs(soln.'));

% cleaning up symbols
syms 'x_s%d' [1 5] % [x y z alpha beta]
syms 'v_s%d' [1 5] 
syms 'a_q%d' [1 3] % acceleration of drone only
soln_rhs = subs(soln_rhs,[q_coord,qdot_coord,vdot_q.'],[x_s,v_s,a_q]);

%% Save files
matlabFunction(soln_rhs(1:3),'File','dyn_eqn_quad','Var',[x_s,v_s,F_l.',g,l_p,m_p,m_q]);
matlabFunction(soln_rhs(4:5),'File','dyn_eqn_pld','Var',[x_s,v_s,a_q,g,l_p,m_p,m_q]);
