function dx = cartpend(x,m,M,L,g,d,u)

Sx = sin(x(3));
Cx = cos(x(3));
B=L*(M + m*(1-Cx^2));

dx(1,1) = x(2);
dx(2,1) = (L/B)*( -m*g*Cx*Sx + m*L*x(4)^2*Sx + u - d*x(2) );
dx(3,1) = x(4);
dx(4,1) = (1/B)*((M+m)*g*Sx - m*L*x(4)^2*Sx*Cx - u*Cx + d*x(2)*Cx);