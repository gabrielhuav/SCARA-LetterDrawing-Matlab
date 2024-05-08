% Limpiar todas las variables, consola y objetos de gráficas
clear;
clc;
cla;

% Definición de los eslabones del robot SCARA
L1 = Link([0 0 8 0 0]);  % Eslabón 1: rotativo, longitud 8
L2 = Link([0 0 10 0 0]); % Eslabón 2: rotativo, longitud 10
L3 = Link([0 0 0 0 1]);  % Eslabón 3: prismatico (a lo largo de Z, no usado aquí)

% Crear el robot SCARA
global R;  % Declara R como una variable global
R = SerialLink([L1, L2, L3], 'name', 'SCARA / Dinámica de Robots / 24I');

% Espacio de trabajo para visualización
global workSpaceLimits;  % Declara workSpaceLimits como una variable global
workSpaceLimits = [-20 20 -20 20 -20 20];

% Posición inicial del robot
global h;  % Declara h como una variable global
if isempty(h) || ~isvalid(h)  % Si la figura no existe o no es válida
    h = figure;  % Crea una nueva figura y guarda su identificador
end
figure(h);  % Establece la figura del robot como la figura actual
R.plot([6, 8, 0],'workspace', workSpaceLimits);


% Longitudes de los eslabones, usadas en la cinemática inversa
l1 = 8;  % Longitud del primer eslabón
l2 = 10; % Longitud del segundo eslabón

tf = 20;    % Tiempo final
ts = 0.1;   % Paso de tiempo
t = 0:ts:tf;    % Vector de tiempo
N = length(t); % Número de puntos en el vector de tiempo
%G %%
hx = zeros(1,N+1);  % Vector de coordenadas x del efector final
hy = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz = zeros(1,N+1);  % Vector de coordenadas z del efector final

M = round(N);  % Límite máximo del primer segmento del dibujo

%% Calcula las posiciones articulares / Primer segmento
for k=1:M
    hx(k) = 15*cosd(k);   % Coordenada x del efector final -4 + 
    hy(k) = 15*sind(k);   % Coordenada y del efector final 7 + 
    hz(k) = 0;
    %pause(0.01); % Pausa breve para visualizar la animación
    A = hx(k);
    B = hy(k);
end

% Dibuja la trayectoria
hold on;
for i = 1:N
    i;
    % Usar la función 'inverse_kinematics' para convertir puntos XY a ángulos de junta
    [q1, q2] = inverse_kinematics(hx(i), hy(i), l1, l2); 
    % plot3(hx, hy, hz, 'r.');
    p = [hx(i), hy(i), hz(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p, 0.2, 'g'); % Dibuja una esfera en el punto actual para visualizar la trayectoria

    R.plot([q1 q2 5],'workspace', workSpaceLimits); % Tercer valor corresponde a d3 que es cero
    drawnow;

end

% *Línea de izquierda a derecha (color verde)*
hx_der = zeros(1,N+1);  % Vector de coordenadas x del efector final (derecha a izquierda)
hy_der = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz_der = zeros(1,N+1);  % Vector de coordenadas z del efector final

for i = 1:N
    hx_der(i) = -16 + (14/N) * i;   % Coordenada x del efector final (derecha a izquierda)
    hy_der(i) = -5.5;               % Coordenada y constante para una línea recta
    hz_der(i) = 0;
end

% Dibuja la trayectoria (derecha a izquierda)
hold on;
for i = 1:(N/1.2)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2); 
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p_der, 0.2, 'g'); % Dibuja una esfera en el punto actual para visualizar la trayectoria (color verde)
    R.plot([q1_der q2_der 5],'workspace', workSpaceLimits); % Color verde
    drawnow;
end

% *Línea de abajo a arriba (color verde)*
hx_der = zeros(1,N+1);  % Vector de coordenadas x del efector final
hy_der = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz_der = zeros(1,N+1);  % Vector de coordenadas z del efector final

for i = 1:N
    hx_der(i) = -4;   % Coordenada x del efector final
    hy_der(i) = -5.5 + (14/N) * i;               % Coordenada y
    hz_der(i) = 0;
end

% Dibuja la trayectoria (de abajo a arriba)
hold on;
for i = 1:N
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2); 
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p_der, 0.2, 'g'); % Dibuja una esfera en el punto actual para visualizar la trayectoria (color verde)
    R.plot([q1_der q2_der 5],'workspace', workSpaceLimits); % Color verde
    drawnow;
end

% H %%
% *Línea de izquierda a derecha (color blanco)*
hx_izq = zeros(1,N+1);  % Vector de coordenadas x del efector final (izquierda a derecha)
hy_izq = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz_izq = zeros(1,N+1);  % Vector de coordenadas z del efector final

for i = 1:N
    hx_izq(i) = -3 + (14/N) * (N-i);   % Coordenada x del efector final (izquierda a derecha)
    hy_izq(i) = -7;               % Coordenada y constante para una línea recta
    hz_izq(i) = 0;
end

% Dibuja la trayectoria (izquierda a derecha)
hold on;
for i = 1:(N/2)
    [q1_izq, q2_izq] = inverse_kinematics(hx_izq(i), hy_izq(i), l1, l2); 
    p_izq = [hx_izq(i), hy_izq(i), hz_izq(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p_izq, 0.2, 'w'); % Dibuja una esfera en el punto actual para visualizar la trayectoria (color blanco)
    R.plot([q1_izq q2_izq 5],'workspace', workSpaceLimits); % Color blanco
    drawnow;
end

% *Línea de abajo a arriba (color blanco)*
hx_der = zeros(1,N+1);  % Vector de coordenadas x del efector final
hy_der = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz_der = zeros(1,N+1);  % Vector de coordenadas z del efector final

for i = 1:N
    hx_der(i) = 8;   % Coordenada x del efector final
    hy_der(i) = -6.5 - (14/N) * i;               % Coordenada y
    hz_der(i) = 0;
end

% Dibuja la trayectoria (de abajo a arriba)
hold on;
for i = 1:(N/3)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2); 
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p_der, 0.2, 'w'); % Dibuja una esfera en el punto actual para visualizar la trayectoria (color blanco)
    R.plot([q1_der q2_der 5],'workspace', workSpaceLimits); % Color blanco
    drawnow;
end

% *Línea de derecha a izquierda (color blanco)*
hx_der = zeros(1,N+1);  % Vector de coordenadas x del efector final (derecha a izquierda)
hy_der = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz_der = zeros(1,N+1);  % Vector de coordenadas z del efector final

for i = 1:N
    hx_der(i) = -3 + (14/N) * (N-i);   % Coordenada x del efector final (derecha a izquierda)
    hy_der(i) = -11.2;               % Coordenada y constante para una línea recta
    hz_der(i) = 0;
end

% Dibuja la trayectoria (derecha a izquierda)
hold on;
for i = 1:(N/2)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2); 
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p_der, 0.2, 'w'); % Dibuja una esfera en el punto actual para visualizar la trayectoria (color blanco)
    R.plot([q1_der q2_der 5],'workspace', workSpaceLimits); % Color blanco
    drawnow;
end
% A %%
% *Línea de derecha a izquierda diagonal hacia abajo*
hx_der = zeros(1,N+1);  % Vector de coordenadas x del efector final (derecha a izquierda)
hy_der = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz_der = zeros(1,N+1);  % Vector de coordenadas z del efector final

for i = 1:N
    hx_der(i) = -6 + (25/N) * i;   % Coordenada x del efector final (derecha a izquierda)
    hy_der(i) = -9 - (11/N) * i;               % Coordenada y constante para una línea recta (-8)
    hz_der(i) = 0;
end

% Dibuja la trayectoria (derecha a izquierda)
hold on;
for i = 1:(N/4)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2); 
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p_der, 0.2, 'r'); % Dibuja una esfera en el punto actual para visualizar la trayectoria (color rojo)
    R.plot([q1_der q2_der 5],'workspace', workSpaceLimits); % Color rojo
    drawnow;
end

% *Línea de derecha a izquierda diagonal hacia arriba (color rojo)*
hx_der = zeros(1,N+1);  % Vector de coordenadas x del efector final (derecha a izquierda)
hy_der = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz_der = zeros(1,N+1);  % Vector de coordenadas z del efector final

for i = 1:N
    hx_der(i) = -6 + (25/N) * i;   % Coordenada x del efector final (derecha a izquierda)
    hy_der(i) = -14.6 + (11/N) * i;               % Coordenada y constante para una línea recta (-8)
    hz_der(i) = 0;
end

% Dibuja la trayectoria (derecha a izquierda) 
hold on;
for i = 1:(N/4)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2); 
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p_der, 0.2, 'r'); % Dibuja una esfera en el punto actual para visualizar la trayectoria (color rojo)
    R.plot([q1_der q2_der 5],'workspace', workSpaceLimits); % Color rojo
    drawnow;
end

% *Línea de abajo a arriba (color rojo)*
hx_der = zeros(1,N+1);  % Vector de coordenadas x del efector final
hy_der = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz_der = zeros(1,N+1);  % Vector de coordenadas z del efector final

for i = 1:N
    hx_der(i) = -3;   % Coordenada x del efector final
    hy_der(i) = -10 - (14/N) * i;               % Coordenada y
    hz_der(i) = 0;
end

% Dibuja la trayectoria (de abajo a arriba)
hold on;
for i = 1:(N/3.8)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2); 
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p_der, 0.2, 'r'); % Dibuja una esfera en el punto actual para visualizar la trayectoria (color rojo)
    R.plot([q1_der q2_der 5],'workspace', workSpaceLimits); % Color rojo
    drawnow;
end

hold off;

% Crear los campos de texto para q1 y q2
global q1_text q2_text;
q1_text = uicontrol('Style', 'edit', 'Position', [20 60 100 20], 'String', '0');
q2_text = uicontrol('Style', 'edit', 'Position', [20 30 100 20], 'String', '0');

% Crear un botón para actualizar la posición del robot
update_button = uicontrol('Style', 'pushbutton', 'Position', [20 90 100 20], 'String', 'Actualizar');

% Definir la función de callback para el botón
set(update_button, 'Callback', @updateRobot);

% Función de callback para actualizar la posición del robot
function updateRobot(src, event)
    % Obtener los campos de texto de la figura
    global q1_text q2_text R workSpaceLimits h;  % Añade workSpaceLimits y h a la lista de variables globales
    q1 = str2double(get(q1_text, 'String')) * (pi / 180);
    q2 = str2double(get(q2_text, 'String')) * (pi / 180);
    z = 0.1;

    % Borrar el robot anterior
    robot_graphics = findobj(h, 'Type', 'Patch');  % Encuentra los objetos gráficos que representan al robot
    delete(robot_graphics);  % Borra solo el robot, no toda la figura

    % Dibujar el robot con los nuevos valores de q1 y q2
    R.plot([q1, q2, z],'workspace', workSpaceLimits);
end

% Función de cinemática inversa
function [q1, q2] = inverse_kinematics(x, y, L1, L2)
    % Calcula theta2
    c2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);

    if c2 < -1 || c2 > 1
        error('El punto [%f, %f] está fuera del alcance del robot.', x, y);
    end
    theta2 = acos(c2);  % Solución principal

    % Calcula theta1
    k1 = L1 + L2 * cos(theta2);
    k2 = L2 * sin(theta2);
    theta1 = atan2(y, x) - atan2(k2, k1);

    % Asigna resultados a las variables de salida
    q1 = theta1;
    q2 = theta2;
end