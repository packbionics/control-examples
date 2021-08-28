clear
g = 9.81;
l = 0.244;
mc = (80+115+75)/1000.0;
mp = 49.0/1000;

A = [zeros(2,2), eye(2);...
	 [0 g*mp/mc; 0 g*(mc+mp)/(mc*l)],zeros(2,2)];

B = [0;0; 1/mc; 1/(l*mc)];

C = [eye(2),zeros(2,2)];

Q = diag([20,10,1,1]);
R = 1;

[K,S,e] = lqr(A,B,Q,R);