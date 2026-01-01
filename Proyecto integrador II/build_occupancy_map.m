function map = build_occupancy_map(scans, poses, params)
% BUILD_OCCUPANCY_MAP - Construye un mapa de ocupacion (Occupancy Grid)
%
% Algoritmo basado en el capitulo 9 de "Probabilistic Robotics" (Thrun et al.)
% Usa el modelo de sensor inverso (inverse sensor model) para actualizar
% las probabilidades de cada celda.
%
% Entradas:
%   scans  - Datos del laser (ranges, angles, timestamps)
%   poses  - Poses del robot (t, x, y, yaw)
%   params - Parametros del mapa (resolution, limites, probabilidades)
%
% Salidas:
%   map    - Estructura con el mapa generado

    % Crear grilla del mapa
    x_range = params.x_min:params.resolution:params.x_max;
    y_range = params.y_min:params.resolution:params.y_max;
    
    n_cols = length(x_range);
    n_rows = length(y_range);
    
    % Inicializar log-odds en 0 (probabilidad 0.5 = desconocido)
    log_odds = zeros(n_rows, n_cols);
    
    % Convertir probabilidades a log-odds
    l_occ = log(params.p_occ / (1 - params.p_occ));
    l_free = log(params.p_free / (1 - params.p_free));
    
    % Limites para evitar overflow
    l_max = 10;
    l_min = -10;
    
    % Interpolar poses a los tiempos de los scans
    poses_x = interp1(poses.t, poses.x, scans.timestamps, 'linear', 'extrap');
    poses_y = interp1(poses.t, poses.y, scans.timestamps, 'linear', 'extrap');
    poses_yaw = interp1(poses.t, unwrap(poses.yaw), scans.timestamps, 'linear', 'extrap');
    
    % Procesar cada scan
    n_scans = min(scans.n, length(scans.ranges));
    
    for i = 1:n_scans
        % Pose del robot en este instante
        robot_x = poses_x(i);
        robot_y = poses_y(i);
        robot_yaw = poses_yaw(i);
        
        % Obtener rangos del scan
        ranges = scans.ranges{i};
        angles = scans.angles;
        
        % Filtrar rangos validos
        valid = isfinite(ranges) & ...
                ranges > scans.range_min & ...
                ranges < scans.range_max;
        
        ranges_valid = ranges(valid);
        angles_valid = angles(valid);
        
        % Para cada rayo valido
        for j = 1:length(ranges_valid)
            r = ranges_valid(j);
            a = angles_valid(j);
            
            % Angulo global del rayo
            global_angle = robot_yaw + a;
            
            % Punto final del rayo (donde impacto)
            end_x = robot_x + r * cos(global_angle);
            end_y = robot_y + r * sin(global_angle);
            
            % Obtener celdas a lo largo del rayo usando Bresenham
            [ray_cells_x, ray_cells_y] = bresenham_line(...
                robot_x, robot_y, end_x, end_y, ...
                x_range, y_range, params.resolution);
            
            % Marcar celdas libres (todas menos la ultima)
            for c = 1:length(ray_cells_x)-1
                col = ray_cells_x(c);
                row = ray_cells_y(c);
                if row >= 1 && row <= n_rows && col >= 1 && col <= n_cols
                    log_odds(row, col) = log_odds(row, col) + l_free;
                    log_odds(row, col) = max(l_min, min(l_max, log_odds(row, col)));
                end
            end
            
            % Marcar celda ocupada (la ultima)
            if ~isempty(ray_cells_x)
                col = ray_cells_x(end);
                row = ray_cells_y(end);
                if row >= 1 && row <= n_rows && col >= 1 && col <= n_cols
                    log_odds(row, col) = log_odds(row, col) + l_occ;
                    log_odds(row, col) = max(l_min, min(l_max, log_odds(row, col)));
                end
            end
        end
    end
    
    % Convertir log-odds a probabilidad
    prob = 1 ./ (1 + exp(-log_odds));
    
    % Guardar resultados
    map.log_odds = log_odds;
    map.prob = prob;
    map.x_range = x_range;
    map.y_range = y_range;
    map.resolution = params.resolution;
    map.params = params;
end


function [cells_x, cells_y] = bresenham_line(x0, y0, x1, y1, x_range, y_range, res)
% BRESENHAM_LINE - Obtiene las celdas a lo largo de una linea
%
% Implementacion simplificada del algoritmo de Bresenham para
% determinar que celdas atraviesa un rayo laser.

    % Convertir coordenadas del mundo a indices de celda
    col0 = round((x0 - x_range(1)) / res) + 1;
    row0 = round((y0 - y_range(1)) / res) + 1;
    col1 = round((x1 - x_range(1)) / res) + 1;
    row1 = round((y1 - y_range(1)) / res) + 1;
    
    % Diferencias
    dx = abs(col1 - col0);
    dy = abs(row1 - row0);
    
    % Direccion del paso
    if col0 < col1
        sx = 1;
    else
        sx = -1;
    end
    
    if row0 < row1
        sy = 1;
    else
        sy = -1;
    end
    
    err = dx - dy;
    
    % Estimar numero maximo de celdas
    max_cells = dx + dy + 1;
    cells_x = zeros(max_cells, 1);
    cells_y = zeros(max_cells, 1);
    
    idx = 0;
    col = col0;
    row = row0;
    
    while true
        idx = idx + 1;
        cells_x(idx) = col;
        cells_y(idx) = row;
        
        if col == col1 && row == row1
            break;
        end
        
        e2 = 2 * err;
        
        if e2 > -dy
            err = err - dy;
            col = col + sx;
        end
        
        if e2 < dx
            err = err + dx;
            row = row + sy;
        end
        
        % Evitar bucle infinito
        if idx >= max_cells
            break;
        end
    end
    
    % Recortar arrays
    cells_x = cells_x(1:idx);
    cells_y = cells_y(1:idx);
end
