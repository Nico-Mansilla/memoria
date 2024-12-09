% Obtener el tamaño de la pantalla
screenSize = get(0, 'ScreenSize');  % [left, bottom, width, height]

% Determinar el tamaño más pequeño entre el ancho y la altura de la pantalla
maxSize = min(screenSize(3), screenSize(4));

% Crear la figura y ajustarla para que sea cuadrada y ocupe el máximo tamaño posible
figure('Position', [0, 0, maxSize, maxSize]);

% Crear el gráfico
x = linspace(0, 2*pi, 100);
y = sin(x);
plot(x, y);

% Asegurarse de que el gráfico ocupe toda la ventana
axis tight;
axis equal;  % Para asegurarse de que los ejes estén escalados igual
