%% ----------------------------------------------------------------
% 3. Reduced-order linear generator cycle model (toy)
%    Idealized free-piston / linear generator thermodynamics
%    Demonstrates real-time capable ODE system for controls
% ----------------------------------------------------------------
fprintf('=== Reduced-order Linear Generator model ===\n');

% Parameters (order-of-magnitude)
m_piston = 5;          % kg
A_p      = 0.01;       % m^2  piston area
V_clear  = 1e-4;       % m^3 clearance
stroke   = 0.1;        % m
k_spring = 1e4;        % N/m  (gas spring approximation)
c_damp   = 20;         % Ns/m
P_atm    = 1e5;
gamma_g  = 1.3;        % effective polytropic

% State: [x, v, P]  (position, velocity, chamber pressure)
% Simple polytropic compression + heat release pulse

odefun = @(t,y) lg_rhs(t,y,m_piston,A_p,V_clear,k_spring,c_damp,P_atm,gamma_g);

y0 = [0.05; 0; 1.5e5];
tspan = [0 0.2];

% Use MATLAB ode45 if available, otherwise fixed-step RK4
if exist('ode45','file')
    [t_lg, Y_lg] = ode45(odefun, tspan, y0);
else
    dt = 1e-5;
    t_lg = (tspan(1):dt:tspan(2))';
    Y_lg = zeros(length(t_lg),3);
    Y_lg(1,:) = y0';
    for i=1:length(t_lg)-1
        k1 = odefun(t_lg(i), Y_lg(i,:)');
        k2 = odefun(t_lg(i)+dt/2, Y_lg(i,:)'+dt/2*k1);
        k3 = odefun(t_lg(i)+dt/2, Y_lg(i,:)'+dt/2*k2);
        k4 = odefun(t_lg(i)+dt,   Y_lg(i,:)'+dt*k3);
        Y_lg(i+1,:) = Y_lg(i,:) + (dt/6)*(k1+2*k2+2*k3+k4)';
    end
end

figure('Name','Linear Generator ROM','Position',[100 100 900 500]);
subplot(2,2,1); plot(t_lg, Y_lg(:,1)*1000,'LineWidth',1.3); ylabel('x [mm]'); title('Piston position'); grid on;
subplot(2,2,2); plot(t_lg, Y_lg(:,2),'LineWidth',1.3); ylabel('v [m/s]'); title('Velocity'); grid on;
subplot(2,2,3); plot(t_lg, Y_lg(:,3)/1e5,'LineWidth',1.3); ylabel('P [bar]'); xlabel('t [s]'); title('Pressure'); grid on;
subplot(2,2,4); plot(Y_lg(:,1)*1000, Y_lg(:,3)/1e5,'LineWidth',1.3); xlabel('x [mm]'); ylabel('P [bar]'); title('P-x diagram'); grid on;

fprintf('\nAll solver demos finished.\n');
fprintf('Key talking points:\n');
fprintf(' - Stiff systems need implicit methods (BDF, Rosenbrock, ...)\n');
fprintf(' - Fixed-step BDF2 is more accurate than BDF1 (Backward Euler) for the same h\n');
fprintf(' - Variable-step BDF (ode15s) adapts step size and order automatically\n');
fprintf(' - DAEs require consistent initialization and index reduction / stabilization\n');
fprintf(' - Real-time control simulation demands reduced-order models that run >> real time\n');

function dydt = lg_rhs(t,y,m,A,Vc,k,c,Patm,gam)
x = y(1); v = y(2); P = y(3);
V = Vc + A*x;                 % instantaneous volume (simplified)
% Force balance
F_press = (P - Patm)*A;
F_spring= -k*(x - 0.05);
F_damp  = -c*v;
a = (F_press + F_spring + F_damp)/m;

% Simple heat-release pulse near TDC (x~0)
Qdot = 0;
if abs(x) < 0.01 && v < 0
    Qdot = 5e4 * exp(-(x/0.005)^2);   % W  (toy)
end

% Polytropic + heat addition (energy balance on chamber)
% d(PV)/dt = (gam-1)*Qdot  approx for ideal gas process
dPdt = (gam-1)*Qdot/V - gam*P/V * (A*v);

dydt = [v; a; dPdt];
end