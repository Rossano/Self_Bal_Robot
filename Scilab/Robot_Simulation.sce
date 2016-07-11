clear; clc;
xdel(winsid()); // close all windows
mode (0);
//  Complex variable
s = poly(0, 's');

// Function to display subplot on the same window
function smallplot(i)
    subplot(3,2,i); xgrid(color('gray'));
    plot(t, x(i,:));
endfunction

function [K, phi] = ackermann (A, B, P)
    phi = poly(P,'x');
    c= coeff(phi);
    phiA = eye(A)*c(1);
    powA = eye(A);
    for i = 2:length(c)
        powA = powA*A;
        phiA = phiA + powA*c(i);
    end
    K = [zeros(1, length(B)-1), 1]*inv(cont_mat(A,B))*phiA;
endfunction

//
//  System values
//
M = 0.02; // Kg

// Gravity
g = 9.81;   // m/s¨2

// Wheels
r = 5*0.021;    // m
m = 0.015;  // Kg
Jw = 1/2*m*r^2;

// Motor
Max_speed = 260;    // rpm
gear_ratio = 50;
Istall = 0.36;      // [A]
Vstall = 6;         // [V]
Tstall = 5*10*0.0071; // [N m] => 10 oz inch Stall torque

R = Vstall/Istall;  // [Ohm] Wind resistance
Km = Tstall/Vstall; // [N m/V] Motor torque gain 
Ke = 1e-6;              // [V s/rad]

// Robot Body
M1 = 0.1;           // [Kg] bottom plate
M2 = 0.1;           // [Kg] top plate
Ma = 0.030;         // [Kg] Arduino
M3 = 0.3;           // [Kg] Batteries
h1 = 0.01;
h2 = 0.050;
h3 = 0.058;
D1 = 0.1;
W1 = 0.2;
D2 = D1;
W2 = W1;
D3 = 0.057;
W3 = 0.09;

// Total Mass
M = M1 + M2 + M3 + Ma;

// Mass Center
// Due to symmetry only on z it is carried out, calling the mass center position l
l = (M1*h1 + M2*h2 + M3*h3)/(M1+M2+M3);

// Robot Inertia
J = M1*(D1^2+W1^2)/12 + M1*h1^2 + M2*(D2^2+W2^2)/12 + M2*h2^2 + M3*(D3^2+W3^2)/12 + M3*h3^2;

// Friction coefficient
b = 0.1;

disp(M, 'Robot Body Mass [Kg] = ');
disp(m, 'Wheel mass [Kg] = ');
disp(l, 'Robot Center of mass (on z) [m] = ');
disp(J, 'Robot inertia [Kg m^2] = ');
disp(Jw, 'Wheel inertia [Kg m^2] = ');
disp(r, 'Robot wheels [m] = ');
disp(Km, 'Motor Couple/current gain [rad/V/s] = ');
disp(gear_ratio, 'Motor reduction gear coefficient = ');
disp(R, 'Motor DC resistance = ');
//disp(tau, 'Motor reactivity = ');
disp(Ke, 'Motor back-emf = ');

// useful constants
beta = 2*(M + m + Jw/r^2);
alpha = J*beta + 2*M*l^2*(m + Jw/r^2);

// System Matrix Definition
A = [0 M*g*l*beta/alpha 2*Km*Ke*(r*beta-M*l)/(R*r^2*alpha) 0;
     1 0 0 0;
     0 (M*l)^2*g/alpha 2*Km*Ke*(M*l*r-J-M*l^2)/(R*r^2*alpha) 0;
     0 0 1 0];
B = [2*Km*(M*l-r*beta)/(R*r*alpha);
    0;
    2*Km*(J+M*l^2-M*l*r)/(R*r*alpha);
    0];
C = [1 0 0 0
    0 1 0 0; 
    0 0 1 0;
    0 0 0 1];
D = [0; 
    0;
    0;
    0];
    
// System Definition
sys = syslin('c', A, B, C, D);

disp(A, 'State Matrix A');
disp(B, 'State Matrix B');

//
//  Desired Behavoir
//
Ts = 1;//0.5;   //  Settling time
OS = 0.15;  //  Overshoot
zeta = sqrt(1/(1+%pi^2/(log(OS))^2));//0.6*(1-OS);
wn = 4.6/zeta/Ts; //2 / zeta / Ts;

l1 = s^2+2*wn*zeta*s+wn^2;
//l1=(s+2-0.5*%i)*(s+2+0.5*%i);
poles_l1 = 5*roots(l1);
//l2 = (s+60)*(s+50);
l2 = (s-real(poles_l1(1)))*(s-real(poles_l1(2)));
l12 = l1*l2;
//Tf = 1000000/l12;
Tf = 1 / l12;
t = 0:0.01:3;   // time axis
y = csim('step', t, Tf);
f1 = figure();
scf(f1);
plot(t,y);
poles = roots(l12);

// Stability check
disp('Stability check: characteristic poly:');
lambda = det(s*eye(4,4)-A)
disp('System eingenvalues:');
roots(lambda)

// Reachability check
disp('Reachability matrix:');
Wr = [B A*B A*A*B A*A*A*B]
disp(det (Wr), 'Inversability of reachablity matrix check (det != 0 system reachable) : ');

//
//  State Feedback Control
//
P = poles;
//Q1 = [-10 -10 -20 -15];
K1 = ackermann(A,B,P);
disp(K1, 'Using Ackmermann formula K = ');
K2 = ppol(A,B,P);
disp(K2, 'Using ppol function K = ');

//
//  Simulating back the system
//
xdel(winsid()); // close all windows
mode (0);
t = 0:0.01:3;
T = C*inv(s*eye(4,4)-(A-B*K1))*B;
Tf=1/(1+0.05*Tf);
disp(Tf,'Final transfer funtion T = ');
Acl = A - B*K2;
disp(roots(determ(s*eye(4,4)-Acl)),"Acl poles : ");
SysCL = syslin('c',Acl, B, C, D);
//yimp = csim('imp',t,Tf);
//ystp = csim('step',t,Tf);
yimp = csim('imp', t, SysCL);
ystp = csim('step', t, SysCL);
//plot(t,yimp,t,ystp);*
f2 = figure();
//plot(t,yimp);
scf(f2);
plot(t, ystp);

Va = zeros(1, length(t));
for i = 1:length(t)
    for j = 1 : 4
        Va(i) = Va(i) - K2(j)*yimp(j,i);
    end;
end;
f3=figure();
plot(t,Va);
