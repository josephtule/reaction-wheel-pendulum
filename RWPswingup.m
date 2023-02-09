clear all;
syms phi_ddot phi_dot phi theta_ddot theta_dot V
syms I_w I_p I_L l L m_w m_L g r_1 r_2 m_m K_v K_t R_i
syms x_1 x_2 x_3
m_p = m_w + m_L + m_m;

% phi_ddot = I_p^-1 * (m_p*l*g*sin(phi) - I_w * theta_ddot)
% F = phi_ddot - I_p^-1 * (m_p*l*g*sin(phi) - I_w * theta_ddot)
t_m = (K_t * V / R_i - K_t^2 * theta_dot / R_i); % torque applied voltage - torque rotation
F(1,1) = phi_ddot - I_p^-1 * (m_p*l*g*sin(phi) - t_m);
F(2,1) = theta_ddot - t_m / I_w

% state space realization
% x = [phi;phi_dot;theta_dot;V]
x = [x_1; x_2; x_3; V]
xdot = [x_2;I_p^-1 * (m_p*l*g*sin(x_1) - (K_t * V / R_i - K_t^2 * x_3 / R_i));(K_t * V / R_i - K_t^2 * x_3 / R_i) / I_w]

for j = 1:length(xdot)
    for k = 1:length(x)
        PD(j,k) = diff(xdot(j),x(k));
    end
end
As = PD(1:3,1:3)
Bs = PD(1:3,4)
I_w = 1/2*m_w*(r_2^2 + r_1^2);
I_L = 1/3 * m_L * L^2;
I_PM = (m_w + m_m)*L^2;
I_p = I_L + I_PM;

m_w = 25/1000; m_m = 230/1000; m_L = 150/1000; L = .25; l = .75*L; r_1 = .15; r_2 = .16; g = 9.81;
params.I_p = eval(subs(I_p,sym([m_w,m_L,L]),[m_w,m_L,L]));
params.I_w = eval(subs(I_w,sym([m_w,r_1,r_2]),[m_w,r_1,r_2]));
params.g = 9.81; params.m_w = m_w; params.m_L = m_L; params.L = L; params.l = l; params.m_m = m_m;
params.m_p = params.m_m + params.m_w + params.m_L; params.umax = 11.1;
params.mu = .025; params.K_v = 35000/12; params.K_t = 60 / 2 / pi / params.K_v; params.R_i = .13; % ohms

phi_e = 0;
A = matlabFunction(As); A = A(params.I_p,params.I_w,params.K_t,params.R_i,...
    params.g,params.l,params.m_L,params.m_m,params.m_w,phi_e)
B = matlabFunction(Bs); B = B(params.I_p,params.I_w,params.K_t,params.R_i)

% K = lqr(A,B,diag([50 10 15]),.5)
K = place(A,B,[-5,-2,-4])
% % K = [-6 -.2]
params.A = A; params.B = B; params.K = K;



options = odeset('RelTol',1e-11,'AbsTol',1e-11);
% tspan = [0,5];
tspan = linspace(0,5,2^12);
x0 = [.5;0;0];

figure()
% [t,x] = ode45(@(t,x) nleoms(t,x,params),tspan,x0,options);
[t,x] = ode45(@(t,x) nleoms(t,x,params),tspan,x0);
subplot(3,1,1)
hold on
plot(t,x(:,1))
if max(x(:,1) > 2.5)
    plot(t,pi*ones(size(t)))
end
grid on
hold off
subplot(3,1,2)
hold on
plot(t,x(:,2))
grid on
hold off
subplot(3,1,3)
plot(t,x(:,3))
grid on
hold off

% r = L*[-sin(x(:,1)) , cos(x(:,1))];
% timemult = 1;
% inc = floor(length(t)/2);
% figure()
% % for k = 1:inc:length(t)
% for k = floor(linspace(1,length(t),inc))
%     plot(r(k,1),r(k,2),'*');
%     hold on
%     xlim([-L,L]); ylim([-L,L])
%     axis square
%     plot(r(1:k,1),r(1:k,2))
%     pause(.00001)
% 
%     if k ~= length(t)
%         clf
%     end
% end


% figure(2)
% [t,x] = ode45(@(t,x) lineoms(t,x,params),tspan,x0,options);
% plot(t,x(:,1))
% hold on
% plot(t,2*pi*ones(size(t)))
% grid on
% hold on

function [A,B] = lin_dyn_SO(F,vars,var_ind)
ddot_ind = var_ind{1}; dot_ind = var_ind{2}; ind = var_ind{3}; u_ind = var_ind{4};

for j = 1:length(F)
    for k = 1:length(vars)
        PD(j,k) = diff(F(j),vars(k));
    end
end

M = PD(:,ddot_ind(1):ddot_ind(end));
D = PD(:,dot_ind(1):dot_ind(end));
S = PD(:,ind(1):ind(end));
W = PD(:,u_ind(1):u_ind(end));
A = simplify([zeros(size(M)), eye(size(M));
    M^-1*(-S), M^-1*(-D)]);
B = simplify([zeros(size(W));M^-1*(-W)]);
end

function dxdt = nleoms(t,x,params)

u = -params.K*x;

maxstab = .25;

if x(1) > maxstab % enable swing up
    if x(2) < 0 % swinging CW
        u = params.umax;
    elseif x(2) > 0 % swinging CCW
%         u = -params.umax;
        u = 0; % can only this way for some reason lol, need to figure out some modulus math so that it can stabilize from the other side as well.
    end
elseif abs(x(1)) <= maxstab % enable stabilization
    if t < 0
        u = 0;
    elseif abs(u) > params.umax
        u = sign(u) * params.umax;
    end
end


dxdt = [x(2);
    params.I_p^-1 * (params.m_p*params.l*params.g*sin(x(1))...
    - (params.K_t * u / params.R_i - params.K_t^2 * x(3) / params.R_i)- params.mu * params.L * x(2));
    (params.K_t * u / params.R_i - params.K_t^2 * x(3) / params.R_i) / params.I_w];

end

function dxdt = lineoms(t,x,params)
u = -params.K*x;
% u = 0;
dxdt = params.A*x+params.B*u;
end

