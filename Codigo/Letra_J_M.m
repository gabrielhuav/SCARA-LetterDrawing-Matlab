% Limpiar todas las variables y la consola
clear;
clc;

% Definición de los eslabones del robot SCARA
L1 = Link([0 0 8 0 0]);  % Eslabón 1: rotativo, longitud 8
L2 = Link([0 0 10 0 0]); % Eslabón 2: rotativo, longitud 10
L3 = Link([0 0 0 0 1]);  % Eslabón 3: prismatico (a lo largo de Z, no usado aquí)

% Crear el robot SCARA
R = SerialLink([L1, L2, L3], 'name', 'SCARA / Dinámica de Robots / 24I');

% Longitudes de los eslabones, usadas en la cinemática inversa
l1 = 8;  % Longitud del primer eslabón
l2 = 10; % Longitud del segundo eslabón

% Espacio de trabajo para visualización
workspace = [-20 20 -20 20 -20 20];

% *Posición inicial del robot (de derecha a izquierda)*
R.plot([-6, 8, 0],'workspace', workspace); % Color azul

tf = 20;    % Tiempo final
ts = 0.1;   % Paso de tiempo
t = 0:ts:tf;    % Vector de tiempo
N = length(t); % Número de puntos en el vector de tiempo

% Robot SCARA dibuja letra "J" en la parte superior
% *Línea de abajo a arriba (color amarillo)*
hx_der = zeros(1,N+1);  % Vector de coordenadas x del efector final
hy_der = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz_der = zeros(1,N+1);  % Vector de coordenadas z del efector final

for i = 1:N
    hx_der(i) = -6;   % Coordenada x del efector final
    hy_der(i) = 8 - (14/N) * i; % Coordenada y del efector final
    hz_der(i) = 0; % Coordenada z del efectpr final
end

% Dibuja la trayectoria (de abajo a arriba)
hold on;
for i = 1:(N/4.5)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2); 
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p_der, 0.2, 'y'); % Dibuja una esfera en el punto actual para visualizar la trayectoria (color amarillo)
    R.plot([q1_der q2_der 5],'workspace', workspace); % Color amarillo
    drawnow;
end

% *Línea de derecha a izquierda (color azul)*
hx_der = zeros(1,N+1);  % Vector de coordenadas x del efector final (derecha a izquierda)
hy_der = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz_der = zeros(1,N+1);  % Vector de coordenadas z del efector final

for i = 1:N
    hx_der(i) = -6 + (14/N) * i;   % Coordenada x del efector final (derecha a izquierda)
    hy_der(i) = 5;               % Coordenada y constante para una línea recta (-8)
    hz_der(i) = 0;
end

% Dibuja la trayectoria (derecha a izquierda)
hold on;
for i = 1:(N/4)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2); 
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p_der, 0.2, 'b'); % Dibuja una esfera en el punto actual para visualizar la trayectoria (color azul)
    R.plot([q1_der q2_der 5],'workspace', workspace); % Color azul
    drawnow;
end

% *Línea de arriba a abajo (color negro)*
hx_der = zeros(1,N+1);  % Vector de coordenadas x del efector final
hy_der = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz_der = zeros(1,N+1);  % Vector de coordenadas z del efector final

for i = 1:N
    hx_der(i) = -2.5;   % Coordenada x del efector final
    hy_der(i) = 5 + (14/N) * i;               % Coordenada y
    hz_der(i) = 0;
end

% Dibuja la trayectoria (de abajo a arriba)
hold on;
for i = 1:(N/2)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2); 
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p_der, 0.2, 'k'); % Dibuja una esfera en el punto actual para visualizar la trayectoria (color negro)
    R.plot([q1_der q2_der 5],'workspace', workspace); % Color negro
    drawnow;
end

% *Línea de derecha a izquierda diagonal hacia arriba (color rojo)*
hx_der = zeros(1,N+1);  % Vector de coordenadas x del efector final (derecha a izquierda)
hy_der = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz_der = zeros(1,N+1);  % Vector de coordenadas z del efector final

for i = 1:N
    hx_der(i) = -4.4 + (14/N) * i;   % Coordenada x del efector final (derecha a izquierda)
    hy_der(i) = 12;               % Coordenada y constante para una línea recta (-8)
    hz_der(i) = 0;
end

% Dibuja la trayectoria (derecha a izquierda)
hold on;
for i = 1:(N/4)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2); 
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p_der, 0.2, 'r'); % Dibuja una esfera en el punto actual para visualizar la trayectoria (color azul)
    R.plot([q1_der q2_der 5],'workspace', workspace); % Color azul
    drawnow;
end



% Robot SCARA dibuja letra M en la parte inferior
% *Línea de abajo a arriba (color amarillo)*
hx_der = zeros(1,N+1);  % Vector de coordenadas x del efector final
hy_der = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz_der = zeros(1,N+1);  % Vector de coordenadas z del efector final

for i = 1:N
    hx_der(i) = -6;   % Coordenada x del efector final
    hy_der(i) = -15 + (14/N) * i;               % Coordenada y
    hz_der(i) = 0;
end

% Dibuja la trayectoria (de abajo a arriba)
hold on;
for i = 1:(N/2)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2); 
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p_der, 0.2, 'y'); % Dibuja una esfera en el punto actual para visualizar la trayectoria (color amarillo)
    R.plot([q1_der q2_der 5],'workspace', workspace); % Color amarillo
    drawnow;
end

% *Línea de derecha a izquierda diagonal hacia abajo (color azul)*
hx_der = zeros(1,N+1);  % Vector de coordenadas x del efector final (derecha a izquierda)
hy_der = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz_der = zeros(1,N+1);  % Vector de coordenadas z del efector final

for i = 1:N
    hx_der(i) = -6 + (14/N) * i;   % Coordenada x del efector final (derecha a izquierda)
    hy_der(i) = -8 - (14/N) * i;               % Coordenada y constante para una línea recta (-8)
    hz_der(i) = 0;
end

% Dibuja la trayectoria (derecha a izquierda)
hold on;
for i = 1:(N/4)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2); 
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p_der, 0.2, 'b'); % Dibuja una esfera en el punto actual para visualizar la trayectoria (color azul)
    R.plot([q1_der q2_der 5],'workspace', workspace); % Color azul
    drawnow;
end

% *Línea de derecha a izquierda diagonal hacia arriba (color rojo)*
hx_der = zeros(1,N+1);  % Vector de coordenadas x del efector final (derecha a izquierda)
hy_der = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz_der = zeros(1,N+1);  % Vector de coordenadas z del efector final

for i = 1:N
    hx_der(i) = -2.4 + (14/N) * i;   % Coordenada x del efector final (derecha a izquierda)
    hy_der(i) = -11.5 + (14/N) * i;               % Coordenada y constante para una línea recta (-8)
    hz_der(i) = 0;
end

% Dibuja la trayectoria (derecha a izquierda)
hold on;
for i = 1:(N/4)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2); 
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p_der, 0.2, 'r'); % Dibuja una esfera en el punto actual para visualizar la trayectoria (color azul)
    R.plot([q1_der q2_der 5],'workspace', workspace); % Color azul
    drawnow;
end


% *Línea de arriba a abajo (color negro)*
hx_der = zeros(1,N+1);  % Vector de coordenadas x del efector final
hy_der = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz_der = zeros(1,N+1);  % Vector de coordenadas z del efector final

for i = 1:N
    hx_der(i) = 1.2;   % Coordenada x del efector final
    hy_der(i) = -8 - (14/N) * i;               % Coordenada y
    hz_der(i) = 0;
end

% Dibuja la trayectoria (de abajo a arriba)
hold on;
for i = 1:(N/2)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2); 
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p_der, 0.2, 'k'); % Dibuja una esfera en el punto actual para visualizar la trayectoria (color negro)
    R.plot([q1_der q2_der 5],'workspace', workspace); % Color negro
    drawnow;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Función de cinemática inversa definida aquí por simplicidad
function [q1, q2] = inverse_kinematics(x, y, L1, L2)
    c2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
    if c2 < -1 || c2 > 1    
        error('El punto [%f, %f] está fuera del alcance del robot.', x, y);
    end
    theta2 = acos(c2);  % Solución principal
    k1 = L1 + L2 * cos(theta2);
    k2 = L2 * sin(theta2);
    theta1 = atan2(y, x) - atan2(k2, k1);
    q1 = theta1;
    q2 = theta2;
end

hold off;