function metrics = evaluate_path(path, map, comp_time, nodes_expanded)
% EVALUATE_PATH - Evalua la calidad de una trayectoria planificada
%
% Calcula metricas cuantitativas para comparar diferentes rutas y
% evaluar el rendimiento de los algoritmos de planificacion.
%
% Entradas:
%   path           : Ruta Nx2 [fila, col]
%   map            : Mapa binario donde se planeo la ruta
%   comp_time      : Tiempo de computo del algoritmo (segundos)
%   nodes_expanded : Numero de nodos expandidos
%
% Salida:
%   metrics : Struct con las siguientes metricas:
%     .num_cells       - Numero de celdas en la ruta
%     .length_m        - Longitud total en metros (asumiendo 0.05 m/celda)
%     .smoothness      - Suavidad de la ruta (suma de cambios de angulo)
%     .avg_clearance   - Distancia media a obstaculos (en celdas)
%     .min_clearance   - Distancia minima a obstaculos
%     .comp_time       - Tiempo de computo
%     .nodes_expanded  - Nodos expandidos
%     .path_valid      - Si la ruta es valida (no cruza obstaculos)
%
% Autores: Antonio Garcia Alcon, Adrian Santero Alonso, Johalex Jose Arrieta Perez

    resolution = 0.05;  % metros por celda
    metrics = struct();
    
    % --- Numero de celdas ---
    metrics.num_cells = size(path, 1);
    
    % --- Longitud total ---
    total_length = 0;
    for i = 2:size(path, 1)
        dx = path(i, 1) - path(i-1, 1);
        dy = path(i, 2) - path(i-1, 2);
        total_length = total_length + sqrt(dx^2 + dy^2);
    end
    metrics.length_cells = total_length;
    metrics.length_m = total_length * resolution;
    
    % --- Suavidad (sum of angular changes) ---
    % Menor valor = ruta mas suave
    smoothness = 0;
    if size(path, 1) >= 3
        for i = 2:size(path, 1) - 1
            v1 = path(i, :) - path(i-1, :);
            v2 = path(i+1, :) - path(i, :);
            
            % Angulo entre vectores consecutivos
            dot_prod = v1(1)*v2(1) + v1(2)*v2(2);
            norm1 = sqrt(v1(1)^2 + v1(2)^2);
            norm2 = sqrt(v2(1)^2 + v2(2)^2);
            
            if norm1 > 0 && norm2 > 0
                cos_angle = dot_prod / (norm1 * norm2);
                cos_angle = max(-1, min(1, cos_angle));  % Clamp por errores numericos
                angle = acos(cos_angle);
                smoothness = smoothness + angle;
            end
        end
    end
    metrics.smoothness = smoothness;
    metrics.avg_smoothness = smoothness / max(1, size(path,1) - 2);
    
    % --- Clearance (distancia a obstaculos) ---
    % Usamos la transformada de distancia del mapa
    dist_map = bwdist(map);  % Distancia de cada celda libre al obstaculo mas cercano
    
    clearances = zeros(size(path, 1), 1);
    for i = 1:size(path, 1)
        r = path(i, 1);
        c = path(i, 2);
        if r >= 1 && r <= size(map, 1) && c >= 1 && c <= size(map, 2)
            clearances(i) = dist_map(r, c);
        else
            clearances(i) = 0;
        end
    end
    
    metrics.avg_clearance = mean(clearances);
    metrics.min_clearance = min(clearances);
    
    % --- Tiempo de computo y nodos ---
    metrics.comp_time = comp_time;
    metrics.nodes_expanded = nodes_expanded;
    
    % --- Validez de la ruta ---
    metrics.path_valid = true;
    for i = 1:size(path, 1)
        r = path(i, 1);
        c = path(i, 2);
        if r < 1 || r > size(map, 1) || c < 1 || c > size(map, 2)
            metrics.path_valid = false;
            break;
        end
        if map(r, c) == 1
            metrics.path_valid = false;
            break;
        end
    end
end
