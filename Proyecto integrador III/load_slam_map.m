function [map_binary, map_info] = load_slam_map()
% LOAD_SLAM_MAP - Carga el mapa generado en el Proyecto Integrador II (SLAM)
%
% Si el archivo del mapa SLAM existe (.mat), lo carga directamente.
% Si no, genera un mapa representativo basado en el entorno del MiR100
% que usamos en los proyectos anteriores (simulando el resultado del
% Occupancy Grid Mapping que implementamos).
%
% Salidas:
%   map_binary : Matriz binaria (0=libre, 1=ocupado)
%   map_info   : Struct con metadatos (resolution, origin, etc.)
%
% Autores: Antonio Garcia Alcon, Adrian Santero Alonso, Johalex Jose Arrieta Perez

    % Intentar cargar el mapa del SLAM si existe
    if isfile('slam_map.mat')
        fprintf('    Cargando mapa SLAM desde slam_map.mat...\n');
        data = load('slam_map.mat');
        if isfield(data, 'occupancy_grid')
            map_binary = data.occupancy_grid;
        elseif isfield(data, 'map_binary')
            map_binary = data.map_binary;
        else
            % Tomar el primer campo disponible
            fields = fieldnames(data);
            map_binary = data.(fields{1});
        end
        
        map_info.resolution = 0.05;  % 5 cm/celda (tipico del MiR100)
        map_info.origin = [0, 0];
        map_info.source = 'SLAM Proyecto II';
        return;
    end
    
    % Si no existe el archivo, generamos un mapa que representa el entorno
    % del MiR100 basado en el mapa de referencia del rosbag.
    % Este mapa simula un entorno de interior tipo oficina/laboratorio
    % con pasillos, habitaciones y obstaculos.
    fprintf('    Archivo slam_map.mat no encontrado.\n');
    fprintf('    Generando mapa representativo del entorno MiR100...\n');
    
    % Parametros del mapa (basados en el mapa del rosbag)
    resolution = 0.05;  % 5 cm por celda
    width_m  = 20;       % 20 metros de ancho
    height_m = 12;       % 12 metros de alto
    
    cols = round(width_m / resolution);   % 400 celdas
    rows = round(height_m / resolution);  % 240 celdas
    
    % Inicializar mapa vacio (todo libre)
    map_binary = zeros(rows, cols);
    
    % --- Paredes exteriores ---
    wall_thickness = 3;  % 15 cm de grosor
    map_binary(1:wall_thickness, :) = 1;              % Pared superior
    map_binary(end-wall_thickness+1:end, :) = 1;      % Pared inferior
    map_binary(:, 1:wall_thickness) = 1;              % Pared izquierda
    map_binary(:, end-wall_thickness+1:end) = 1;      % Pared derecha
    
    % --- Pasillo principal horizontal (centro del mapa) ---
    % Paredes del pasillo
    corridor_y1 = round(rows * 0.4);   % Pared superior del pasillo
    corridor_y2 = round(rows * 0.6);   % Pared inferior del pasillo
    
    % Pared superior del pasillo (con huecos para puertas)
    map_binary(corridor_y1:corridor_y1+2, 1:cols) = 1;
    % Huecos (puertas) en la pared superior
    map_binary(corridor_y1:corridor_y1+2, 60:75) = 0;
    map_binary(corridor_y1:corridor_y1+2, 150:165) = 0;
    map_binary(corridor_y1:corridor_y1+2, 250:265) = 0;
    map_binary(corridor_y1:corridor_y1+2, 340:355) = 0;
    
    % Pared inferior del pasillo (con huecos)
    map_binary(corridor_y2:corridor_y2+2, 1:cols) = 1;
    map_binary(corridor_y2:corridor_y2+2, 80:95) = 0;
    map_binary(corridor_y2:corridor_y2+2, 180:195) = 0;
    map_binary(corridor_y2:corridor_y2+2, 300:315) = 0;
    
    % --- Habitaciones superiores ---
    % Division vertical entre habitaciones
    map_binary(wall_thickness:corridor_y1, 120:122) = 1;
    map_binary(wall_thickness:corridor_y1, 200:202) = 1;
    map_binary(wall_thickness:corridor_y1, 300:302) = 1;
    
    % Puertas en divisiones verticales superiores
    map_binary(round(rows*0.2):round(rows*0.2)+12, 120:122) = 0;
    map_binary(round(rows*0.15):round(rows*0.15)+12, 200:202) = 0;
    map_binary(round(rows*0.25):round(rows*0.25)+12, 300:302) = 0;
    
    % --- Habitaciones inferiores ---
    map_binary(corridor_y2:rows-wall_thickness, 100:102) = 1;
    map_binary(corridor_y2:rows-wall_thickness, 220:222) = 1;
    map_binary(corridor_y2:rows-wall_thickness, 330:332) = 1;
    
    % Puertas inferiores
    map_binary(round(rows*0.7):round(rows*0.7)+12, 100:102) = 0;
    map_binary(round(rows*0.75):round(rows*0.75)+12, 220:222) = 0;
    map_binary(round(rows*0.7):round(rows*0.7)+12, 330:332) = 0;
    
    % --- Obstaculos tipo muebles/equipos (simulando un entorno real) ---
    % Estos representan lo que el laser del MiR100 habria detectado
    
    % Mesas/estanterias en habitaciones superiores
    map_binary(15:25, 30:50) = 1;
    map_binary(15:25, 70:85) = 1;
    map_binary(20:30, 140:160) = 1;
    map_binary(10:18, 230:260) = 1;
    map_binary(25:35, 320:340) = 1;
    
    % Muebles en habitaciones inferiores
    map_binary(170:180, 40:60) = 1;
    map_binary(175:185, 130:150) = 1;
    map_binary(165:175, 250:275) = 1;
    map_binary(180:190, 350:370) = 1;
    
    % Columnas en el pasillo (obstaculos puntuales)
    map_binary(corridor_y1+5:corridor_y2-5, 195:198) = 1;
    
    % Informacion del mapa
    map_info.resolution = resolution;
    map_info.origin = [0, 0];
    map_info.width_m = width_m;
    map_info.height_m = height_m;
    map_info.source = 'Generado (basado en entorno MiR100)';
    
    fprintf('    Mapa generado: entorno tipo oficina/laboratorio\n');
end
