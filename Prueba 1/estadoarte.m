% Importar CasADi
addpath('C:\Users\nicolas\Desktop\Memoria\casadi-3.6.5-windows64-matlab2018b')
import casadi.*

x = MX.sym('x',2); % Two states
p = MX.sym('p');   % Free parameter

% Expression for ODE right-hand side
rhs = [1-x(2)^2*x(1)-x(2)+2*tanh(p);x(1)];

% Crear función para rhs
rhs_func = Function('rhs_func', {x, p}, {rhs});

% ODE declaration with free parameter
ode = struct('x',x,'p',p,'ode',rhs);

% Resto del código...

% Evaluar la función en un rango de valores
x_values = linspace(-2, 2, 100);
rhs_values = zeros(length(x_values), 2);
for i = 1:length(x_values)
    rhs_values(i, :) = full(rhs_func([x_values(i); 0], 0.2));
end

% Trazar la función
figure;
plot(x_values, rhs_values);
title('rhs');