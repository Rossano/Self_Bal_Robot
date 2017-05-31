//  Complex variable
s = poly(0, 's');

//
//  System values
//
M = 0.02; // Kg

// Gravity
g = 9.81;   // m/s¨2

// Wheels
r = 0.02; //0.021;    // m
m = 0.019;  // Kg
Jw = 1/2*m*r^2;

// Motor
Max_speed = 260;    // rpm
gear_ratio = 50;
Istall = 0.36;      // [A]
I0 = 0.04;
Vstall = 6;         // [V]
//Tstall = 10*5*10*0.0071; // [N m] => 10 oz inch Stall torque
Tstall = 10 * 0.0071; // [N m] => 10 oz inch torque

//R = Vstall/Istall;  // [Ohm] Wind resistance
R = Vstall/I0;
//Km = Tstall/Vstall; // [N m/V] Motor torque gain 
Km = Tstall / (Istall-I0);  // [N m / A]
//Ke = 1e-6;              // [V s/rad]
Ke = 0.8e-3 * 60 / 2/ %pi;

// Robot Body
M1 = 0.182;           // [Kg] bottom plate
M2 = 0.102;           // [Kg] top plate
Ma = 0.030;         // [Kg] Arduino
M3 = 0.3; //0.162;           // [Kg] Batteries
h1 = 0.01;
h2 = 0.5; //0.076;
h3 = 0.08;
D1 = 0.100;
W1 = 0.180;
Z1 = 0.005;
D2 = D1;
W2 = W1;
Z2 = Z1;
D3 = 0.038;
W3 = 0.09;
Z3 = 0.016;

// Total Mass
M = M1 + M2 + M3 + Ma;

// Mass Center
// Due to symmetry only on z it is carried out, calling the mass center position l
l = (M1*h1 + M2*h2 + M3*h3)/(M1+M2+M3);

// Robot Inertia
J = M1*(D1^2+W1^2)/12 + M1*h1^2 + M2*(D2^2+W2^2)/12 + M2*h2^2 + M3*(D3^2+W3^2)/12 + M3*h3^2;

// Friction coefficient
b = 0.1;

disp(M, 'Robot Body Mass [Kg] M = ');
disp(m, 'Wheel mass [Kg] m = ');
disp(l, 'Robot Center of mass (on z) [m] l = ');
disp(J, 'Robot inertia [Kg m^2] J = ');
disp(Jw, 'Wheel inertia [Kg m^2] Jw = ');
disp(r, 'Robot wheels [m] r = ');
disp(Km, 'Motor Couple/current gain Km [N m/A] = ');
disp(gear_ratio, 'Motor reduction gear coefficient = ');
disp(R, 'Motor DC resistance R = ');
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
C = [1 0 0 0];
    //0 1 0 0; 
    //0 0 1 0;
    //0 0 0 1];
D = [0]; 
    //0;
    //0;
    //0];
    
// System Definition
sys = syslin('c', A, B, C, D);

disp(A, 'State Matrix A');
disp(B, 'State Matrix B');

//
//  Desired Behavoir
//
Ts = 100;//0.5;   //  Settling time
OS = 0.15;  //  Overshoot
zeta = sqrt(1/(1+%pi^2/(log(OS))^2));//0.6*(1-OS);
wn = 4.6/zeta/Ts; //2 / zeta / Ts;

l1 = s^2+2*wn*zeta*s+wn^2;
poles_l1 = 5*roots(l1);
l2 = (s-real(poles_l1(1)))*(s-real(poles_l1(2)));
l12 = l1*l2;
Tf = 1 / l12;

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

K2 = ppol(A,B,P);
disp(K2, 'Using ppol function K = ');

Ku = 1;///(C*(inv(A-B*K2)*B));

//
// Linear Quadratic Compensator
//

nstates = 4;
// LQ classic notation x'Qx + u'Ru
Q = diag([100, 100, 100, 100]);     // 4 states 1 output
R = 1;
Big = sysdiag(Q,R);
// Now carry out C1 and D12
[w,wp] = fullrf(Big);
C1 = wp(:,1:nstates);
D12 = wp(:,nstates+1:$);
Plant = syslin('c', A, B, C1, D12); // The plant (the robot)) 
[K,XX] = lqr(Plant);
disp(spec(A+B*K), "Stability check (> 0)");     // check stability
disp(norm(A'*XX*A-(A'*XX*B)*pinv(B'*XX*B+R)*(B'*XX*A)+Q-XX,1), "Riccati check"); // Riccati check

ref = 0;
dt = 0.01;
tfin = 1.5;
t = 0:dt:tfin;
Acl = A+B*K;
Bcl = B;//*Ku;
sysCL = syslin('d', Acl, Bcl, C);
//u = ones(1, length(t));
//X = ltitr(Acl, Bcl, u);

X0=[3/180*%pi 0 0 0];
X=zeros(4,length(t));
X(:,1) = X0';
Vref = 0;
//Va = zeros(length(t),1);
for i=2:length(t)
//    Va(i) = Vref -K2(1)*X(1,i)-K2(2)*X(2,i)-K2(3)*X(3,i)-K2(4)*X(4,i);
//    X(:,i+1)=A*X(:,i)+B*Va(i); 
    //Va(i-1) = Vref - (K2(1)*X(1,i-1)+K2(2)*X(2,i-1)+K2(3)*X(3,i-1)+K2(4)*X(4,i-1));
    Va(i-1) = Vref + (K(1)*X(1,i-1)+K(2)*X(2,i-1)+K(3)*X(3,i-1)+K(4)*X(4,i-1));
    //if(Va(i-1) > 5.0) Va(i-1) = 5.0
    //else if (Va(i-1) < -5.0) Va(i-1) = -5.0            
    //    end
    //end
    
    X(:,i)=A*X(:,i-1)+B*Va(i-1);
end
Va(length(t)) = Va(length(t)-1);
plot(t,Va);
//plot(t,X(1,:));
//plot(t,u);
