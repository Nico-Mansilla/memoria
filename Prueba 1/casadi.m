% Importar CasADi
addpath('C:\Users\nicolas\Desktop\Memoria\casadi-3.6.5-windows64-matlab2018b')
import casadi.*

% Definir variables simbólicas
x = MX.sym('x');
y = MX.sym('y');

% Definir la función objetivo
f = x^2 + y^2;

% Definir las restricciones
g = x + y - 1;

% Crear un objeto NLP (Nonlinear Programming)
nlp = struct('x', [x; y], 'f', f, 'g', g);

% Crear un objeto solver
opts = struct('ipopt', struct('print_level', 0), 'print_time', 0);
solver = nlpsol('solver', 'ipopt', nlp, opts);

% Resolver el problema de optimización
res = solver('x0', [0; 0], 'lbx', [-inf; -inf], 'ubx', [inf; inf], 'lbg', 0, 'ubg', 0);

% Mostrar la solución
solution = full(res.x);
disp(solution);

