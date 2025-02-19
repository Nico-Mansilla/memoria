% Parámetros de la distribución
media = 0;                  % Media de la distribución (centrada en 0)
varian = 2;                 % Varianza deseada (puedes cambiar este valor)
varian2 = 3;
varian3 = 1;
num_muestras = 1000000;      % Número de muestras

% Generar las muestras de la distribución gaussiana
datos = media + sqrt(varian) * randn(num_muestras, 1);
datos2 = media + sqrt(varian2) * randn(num_muestras, 1);
datos3 = media + sqrt(varian3) * randn(num_muestras, 1);
datos_mean = mean([datos,datos2,datos3],2);

% Calcular la varianza real de los datos generados
varianza_real = var(datos);

% Graficar el histograma
figure;
histogram(datos, 30, 'Normalization', 'pdf'); % 30 bins, normalizado a pdf
hold on;

% Graficar la curva de la distribución normal teórica
x = linspace(min(datos), max(datos), 100);
y = (1/(sqrt(2*pi*varian))) * exp(-(x - media).^2 / (2 * varian));
plot(x, y, 'r', 'LineWidth', 2);

% Título y etiquetas
title('Histograma de una distribución Gaussiana');
xlabel('Valor');
ylabel('Densidad de probabilidad');
legend('Histograma', 'Distribución teórica', 'Location', 'Best');

% Mostrar la varianza real en la consola
disp(['La varianza real de los datos generados es: ', num2str(varianza_real)]);
