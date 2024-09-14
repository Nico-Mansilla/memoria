% Definición de las funciones de transferencia del proceso y del controlador P
s = tf('s');  % Variable compleja 's'

% Función de transferencia del proceso (planta)
Gp = 1 / (s^2 + 2*s + 1);

% Función de transferencia del controlador PI
Kp = 1;  % Ganancia proporcional
Ki = 0.1;  % Ganancia integral
Gc = Kp + Ki/s;

% Lazo de control cerrado
Gc_feedback = feedback(Gc*Gp, 1);

% Resto del código...

% Referencia a step
t = 0:0.01:200;  % Vector de tiempo
r = ones(size(t));  % Referencia a step de amplitud 1
r(t > 100) = 0.5;  % Cambiar la referencia a 0.5 a partir del segundo 100

% Simulación del sistema con la referencia a step
[y, t] = lsim(Gc_feedback, r, t);

% Gráfica de la respuesta del sistema
plot(t, r, 'r--', t, y, 'b', 'LineWidth', 2);
xlabel('Tiempo');
ylabel('Respuesta');
title('Respuesta del sistema a una referencia a step');
legend('Referencia', 'Respuesta del sistema');
