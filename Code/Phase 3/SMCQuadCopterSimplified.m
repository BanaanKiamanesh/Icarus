clc;
clear;
close all;

%% Initial Values
SimTime = 40;
dt = 0.005;
t = 0:dt:SimTime;
StepNum = length(t);

X = zeros(12, StepNum);
X(11, 1) = 1;

U = zeros(4, StepNum);
Ux = zeros(1, StepNum);
Uy = zeros(1, StepNum);

U(:, 1) = 4*ones(4, 1);

g = 9.81;
m = 0.650;
l = 0.23;
Ir = 6e-5;
Ixx = 7.5e-3;
Iyy = Ixx;
Izz = 1.3e-2;
k = 3.13e-5;
d = 7.5e-7;

a(1) = (Iyy - Izz) / Ixx;
a(2) = -Ir / Ixx;
a(3) = (Izz - Ixx) / Iyy;
a(4) = -Ir / Iyy;
a(5) = (Ixx - Iyy) / Izz;
b(1) = l / Ixx;
b(2) = l / Iyy;
b(3) = l / Izz;

SPhi   = zeros(1, StepNum);
STheta = zeros(1, StepNum);
SPsi   = zeros(1, StepNum);
SZ     = zeros(1, StepNum);
SY     = zeros(1, StepNum);
SX     = zeros(1, StepNum);

Omega  = zeros(5, StepNum);

Zeta = [0.35804, 0.50581, 0.34049, 0.61075, 0.61075, 0.61075];
Kappa = [8.0568, 13.6547, 1.8914, 1.358775, 5.2608, 5.0176];

Phid = zeros(1, StepNum);
Thetad = zeros(1, StepNum);

% Disturbance Bounds
Dl = -0.65;
Du = 0.65;

D1 = (Du - Dl) / 2;
D2 = (Du + Dl) / 2;

for i = 1:StepNum-1

    % Trajectory Gen
    [Xd, dXd, ddXd] = Trajectory(t(i));

    % Disturbance Term
    Disturb = Disturbance(t(i));

    Omega(1, i) = sqrt(U(1, i) / (4*k) - (U(3, i) / (2*k*l)) - (U(4, i) / (4*d)));
    Omega(2, i) = sqrt(U(1, i) / (4*k) - (U(2, i) / (2*k*l)) + (U(4, i) / (4*d)));
    Omega(3, i) = sqrt(U(1, i) / (4*k) - (U(3, i) / (2*k*l)) - (U(4, i) / (4*d)));
    Omega(4, i) = sqrt(U(1, i) / (4*k) + (U(2, i) / (2*k*l)) + (U(4, i) / (4*d)));
    Omega(5, i) = -Omega(1, i) + Omega(2, i) - Omega(3, i) + Omega(4, i);

    Phid(i)   = -asin(Uy(i));
    Thetad(i) = asin(Ux(i) / cos(Phid(i)));

    SPhi(i + 1)   =        - X(2, i)  + Zeta(1) * (Phid(i)   - X(1,  i));
    STheta(i + 1) =        - X(4, i)  + Zeta(2) * (Thetad(i) - X(3,  i));
    SPsi(i + 1)   = dXd(4) - X(6, i)  + Zeta(3) * (Xd(4)     - X(5,  i));
    SX(i + 1)     = dXd(1) - X(8, i)  + Zeta(4) * (Xd(1)     - X(7,  i));
    SY(i + 1)     = dXd(2) - X(10, i) + Zeta(5) * (Xd(2)     - X(9,  i));
    SZ(i + 1)     = dXd(3) - X(12, i) + Zeta(6) * (Xd(3)     - X(11, i));

    U(1, i + 1) = m / (cos(X(1, i)) * cos(X(3, i))) * (Kappa(4)*signum(SZ(i)) + g + Zeta(4)*(dXd(3) - X(12, i)) + ddXd(3) + (D2 - D1 * signum(SZ(i))));
    U(2, i + 1) = (1/b(1)) * (Kappa(1) * signum(SPhi(i))    - a(1)*X(4, i)*X(6, i) - X(4, i)*a(2)*Omega(5, i) - Zeta(1)*(X(2, i)) + (D2 - D1 * signum(SPhi(i))));
    U(3, i + 1) = (1/b(2)) * (Kappa(2) * signum(STheta(i))  - a(3)*X(2, i)*X(6, i) - X(2, i)*a(4)*Omega(5, i) - Zeta(2)*(X(4, i)) + (D2 - D1 * signum(STheta(i))));
    U(4, i + 1) = (1/b(3)) * (Kappa(3) * signum(SPsi(i))    - a(5)*X(4, i)*X(2, i) + Zeta(3)*(dXd(4) - X(6, i)) + ddXd(4) + (D2 - D1 * signum(SPsi(i))));
    Ux(i + 1)   = (m / U(1, i)) * (Kappa(5)*signum(SX(i)) + Zeta(5)*(dXd(1) - X(8, i)) + ddXd(1) + (D2 - D1 * signum(SX(i))));
    Uy(i + 1)   = (m / U(1, i)) * (Kappa(6)*signum(SY(i)) + Zeta(6)*(dXd(2) - X(10, i))+ ddXd(2) + (D2 - D1 * signum(SY(i))));

    X(1, i + 1)  = X(1, i)  + dt * (X(2, i) + deg2rad(rand));
    X(2, i + 1)  = X(2, i)  + dt * (X(4, i)*X(6, i)*a(1) + X(4, i)*a(2)*Omega(5, i) + b(1)*U(2, i) + Disturb);
    X(3, i + 1)  = X(3, i)  + dt * (X(4, i) + deg2rad(rand));
    X(4, i + 1)  = X(4, i)  + dt * (X(2, i)*X(6, i)*a(3) + X(2, i)*a(4)*Omega(5, i) + b(2)*U(3, i) + Disturb);
    X(5, i + 1)  = X(5, i)  + dt * (X(6, i) + deg2rad(rand));
    X(6, i + 1)  = X(6, i)  + dt * (X(4, i)*X(6, i)*a(5) + b(3)*U(4, i) + Disturb);
    X(7, i + 1)  = X(7, i)  + dt * (X(8, i));
    X(8, i + 1)  = X(8, i)  + dt * (Ux(i + 1)*(1/m)*U(1, i) + Disturb);
    X(9, i + 1)  = X(9, i)  + dt * (X(10, i));
    X(10, i + 1) = X(10, i) + dt * (Uy(i + 1)*(1/m)*U(1, i) + Disturb);
    X(11, i + 1) = X(11, i) + dt * (X(12, i));
    X(12, i + 1) = X(12, i) + dt * (-g + (((cos(X(1, i))*cos(X(3, i))*U(1, i))/m)) + Disturb);
end

figure
subplot(2, 1, 1)
plot(t, Phid); xlabel('Time(sec)');ylabel('Phi Desired')
subplot(2, 1, 2)
plot(t, Thetad); xlabel('Time(sec)');ylabel('Theta Desired')

figure
subplot(3, 4, 1)
plot(t, X(1, :)); xlabel('Time(sec)'); ylabel('phi')
subplot(3, 4, 2)
plot(t, X(2, :)); xlabel('Time(sec)'); ylabel('p')
subplot(3, 4, 3)
plot(t, X(3, :)); xlabel('Time(sec)'); ylabel('theta')
subplot(3, 4, 4)
plot(t, X(4, :)); xlabel('Time(sec)'); ylabel('q')
subplot(3, 4, 5)
plot(t, X(5, :)); xlabel('Time(sec)'); ylabel('ksi')
subplot(3, 4, 6)
plot(t, X(6, :)); xlabel('Time(sec)'); ylabel('r')
subplot(3, 4, 7)
plot(t, X(7, :)); xlabel('Time(sec)'); ylabel('X')
subplot(3, 4, 8)
plot(t, X(8, :)); xlabel('Time(sec)'); ylabel('u')
subplot(3, 4, 9)
plot(t, X(9, :)); xlabel('Time(sec)'); ylabel('Y')
subplot(3, 4, 10)
plot(t, X(10, :)); xlabel('Time(sec)'); ylabel('V')
subplot(3, 4, 11)
plot(t, X(11, :)); xlabel('Time(sec)'); ylabel('Z')
subplot(3, 4, 12)
plot(t, X(12, :)); xlabel('Time(sec)'); ylabel('w')