clc
clear
close all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parámetros de conexión MQTT
% topic = 'Matlab'; % Tema al que se publicará
% mqttClient = mqttclient("mqtt://192.168.137.1:1883");% Dirección del broker MQTT

% % Suscripción al tema Matlab
% try
%     subscribe(mqttClient, topic);
%     disp(['Suscrito al tema: ' topic]);
% catch ME
%     disp(['Error al suscribirse al tema: ' ME.message]);
%     return;
% end
% Fin de Parámetros de conexión MQTT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONSTRUCCION Y SIMULACION CON ROBOTICS TOOLBOX
% Definir las articulaciones
L1 = Link('d', 158, 'a', 200, 'alpha', 0);
L2 = Link('d', 43, 'a', 200, 'alpha', 0);
L3 = Link('d', 52, 'a', 0, 'alpha', 0);

% Simulacion del robot SCARA con sus dimensiones
global R;
R = SerialLink([L1, L2, L3], 'name', 'SCARA / Dinámica de Robots / 24I');

% Espacio de trabajo para visualización
global workSpaceLimits;  % Declara workSpaceLimits como una variable global
workSpaceLimits = [-500 500 -500 500 -500 500];

% Posición inicial del robot
global h;  % Declara h como una variable global
if isempty(h) || ~isvalid(h)  % Si la figura no existe o no es válida
    h = figure;  % Crea una nueva figura y guarda su identificador
end
figure(h);  % Establece la figura del robot como la figura actual
R.plot([0, 0, 0],'workspace', workSpaceLimits);
% FIN DE LA CONSTRUCCION Y SIMULACION CON ROBOTICS TOOLBOX
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Longitudes de los eslabones, usadas en la cinemática inversa
l1 = L1.a;
l2 = L2.a;

tf = 20;    % Tiempo final
ts = 0.1;   % Paso de tiempo
t = 0:ts:tf;    % Vector de tiempo
N = length(t); % Número de puntos en el vector de tiempo

% Línea de abajo a arriba
hx_der = zeros(1,N+1);  % Vector de coordenadas x del efector final
hy_der = zeros(1,N+1);  % Vector de coordenadas y del efector final
hz_der = zeros(1,N+1);  % Vector de coordenadas z del efector final

for i = 1:N
    hx_der(i) = -200;   % Coordenada x del efector final
    hy_der(i) = -200 + (400/N) * i;               % Coordenada y
    hz_der(i) = 0;
end

% Dibuja la trayectoria (de abajo a arriba)
hold on;
for i = 1:N
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2); 

    % Resultados
    fprintf("q1 = %.11f\n", q1_der);
    fprintf("q2 = %.11f\n", q2_der);

    % Empaquetamiento datos en estructura JSON
    data = struct('q1', q1_der, 'q2', q2_der);
    jsonData = jsonencode(data);

    % Publicación del mensaje JSON al tema
    message = jsonData;
    %write(mqttClient, topic, message);  % Descomenta esta línea para enviar el mensaje

    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Obtiene el punto actual de la trayectoria
    plot_sphere(p_der, 10, 'r'); % Dibuja una esfera en el punto actual para visualizar la trayectoria (color verde)
    R.plot([q1_der q2_der 0],'workspace', workSpaceLimits); % Color verde
    drawnow;
end
hold off;

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCIÓN PARA ACTUALIZAR LA POSICIÓN DEL ROBOT CON POO

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
% FIN DE LA FUNCIÓN PARA ACTUALIZAR LA POSICIÓN DEL ROBOT CON POO
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
