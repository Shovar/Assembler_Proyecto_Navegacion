function [path, nodes_expanded, total_cost] = dstar_lite(map, start, goal, old_path)
% DSTAR_LITE - Algoritmo D* Lite para replanificacion ante obstaculos dinamicos
%
% Implementa una version simplificada del algoritmo D* Lite
% (Koenig & Likhachev, 2002) para recalcular la ruta cuando aparecen
% nuevos obstaculos en el mapa. A diferencia de A* que busca desde cero,
% D* Lite reutiliza informacion de la busqueda anterior para ser mas
% eficiente en la replanificacion.
%
% El algoritmo trabaja "hacia atras" (desde la meta hacia el inicio),
% lo que permite actualizar solo los nodos afectados por el cambio
% en el mapa sin tener que recalcular toda la ruta.
%
% Entradas:
%   map      : Mapa actualizado con el nuevo obstaculo (0=libre, 1=ocupado)
%   start    : [fila, col] posicion actual del robot
%   goal     : [fila, col] posicion de la meta
%   old_path : Ruta anterior (para detectar invalidez y guiar la busqueda)
%
% Salidas:
%   path           : Nueva ruta Nx2 [fila, col]
%   nodes_expanded : Nodos expandidos en la replanificacion
%   total_cost     : Costo de la nueva ruta
%
% Referencias:
%   Koenig, S., & Likhachev, M. (2002). D* Lite. Proceedings of the
%   AAAI Conference on Artificial Intelligence (AAAI), 476-483.
%
%   Stentz, A. (1994). Optimal and Efficient Path Planning for
%   Partially-Known Environments. Proceedings of the IEEE International
%   Conference on Robotics and Automation (ICRA), 3310-3317.
%
% Autores: Antonio Garcia Alcon, Adrian Santero Alonso, Johalex Jose Arrieta Perez

    [rows, cols] = size(map);
    
    % Validaciones
    if map(start(1), start(2)) == 1
        % Si el robot esta en un obstaculo (caso extremo), buscar celda libre cercana
        start = find_free_cell(map, start);
        if isempty(start)
            path = [];
            nodes_expanded = 0;
            total_cost = inf;
            return;
        end
    end
    
    if map(goal(1), goal(2)) == 1
        goal = find_free_cell(map, goal);
        if isempty(goal)
            path = [];
            nodes_expanded = 0;
            total_cost = inf;
            return;
        end
    end
    
    % --- Paso 1: Detectar que celdas de la ruta antigua estan bloqueadas ---
    blocked_cells = [];
    if ~isempty(old_path)
        for i = 1:size(old_path, 1)
            r = old_path(i, 1);
            c = old_path(i, 2);
            if r >= 1 && r <= rows && c >= 1 && c <= cols
                if map(r, c) == 1
                    blocked_cells = [blocked_cells; r, c];
                end
            end
        end
    end
    
    fprintf('      Celdas bloqueadas en ruta anterior: %d\n', size(blocked_cells, 1));
    
    % --- Paso 2: Inicializar D* Lite ---
    % rhs(s) y g(s) para cada celda
    % En D* Lite, la busqueda va de META a INICIO (busqueda hacia atras)
    
    g_val = inf(rows, cols);
    rhs = inf(rows, cols);
    
    % La meta tiene rhs = 0
    rhs(goal(1), goal(2)) = 0;
    
    % km: ajuste de la heuristica por movimiento del robot
    km = 0;
    
    % Movimientos 8-conectividad
    moves = [
        -1,  0, 1.0;
         1,  0, 1.0;
         0, -1, 1.0;
         0,  1, 1.0;
        -1, -1, sqrt(2);
        -1,  1, sqrt(2);
         1, -1, sqrt(2);
         1,  1, sqrt(2);
    ];
    
    % --- Paso 3: Cola de prioridad ---
    % Cada entrada: [fila, col, key1, key2]
    % key = [min(g, rhs) + h + km, min(g, rhs)]
    
    key_goal = calculate_key(goal, start, g_val, rhs, km);
    priority_queue = [goal(1), goal(2), key_goal(1), key_goal(2)];
    
    in_queue = false(rows, cols);
    in_queue(goal(1), goal(2)) = true;
    
    nodes_expanded = 0;
    max_expansions = rows * cols;  % Limite de seguridad
    
    % --- Paso 4: Compute Shortest Path (bucle principal de D* Lite) ---
    while ~isempty(priority_queue)
        % Verificar condicion de parada
        key_start = calculate_key(start, start, g_val, rhs, km);
        
        % Encontrar el nodo con menor key en la cola
        [~, min_idx] = min(priority_queue(:, 3) * 1e6 + priority_queue(:, 4));
        u = priority_queue(min_idx, 1:2);
        key_u = priority_queue(min_idx, 3:4);
        
        % Condicion de parada: la key del inicio es <= que la menor key
        % y g(start) == rhs(start)
        if compare_keys(key_start, key_u) <= 0 && ...
                g_val(start(1), start(2)) == rhs(start(1), start(2))
            break;
        end
        
        % Extraer u de la cola
        priority_queue(min_idx, :) = [];
        in_queue(u(1), u(2)) = false;
        
        nodes_expanded = nodes_expanded + 1;
        if nodes_expanded > max_expansions
            warning('D* Lite: limite de expansiones alcanzado');
            break;
        end
        
        ur = u(1);
        uc = u(2);
        
        key_new = calculate_key(u, start, g_val, rhs, km);
        
        if compare_keys(key_u, key_new) < 0
            % Key desactualizada, reinsertar con nueva key
            priority_queue = [priority_queue; ur, uc, key_new(1), key_new(2)];
            in_queue(ur, uc) = true;
            
        elseif g_val(ur, uc) > rhs(ur, uc)
            % Nodo localmente sobreestimado -> actualizar
            g_val(ur, uc) = rhs(ur, uc);
            
            % Actualizar todos los predecesores (vecinos)
            for i = 1:size(moves, 1)
                nr = ur + moves(i, 1);
                nc = uc + moves(i, 2);
                
                if nr < 1 || nr > rows || nc < 1 || nc > cols
                    continue;
                end
                if map(nr, nc) == 1
                    continue;
                end
                
                % Verificar corte de esquinas en diagonales
                if moves(i, 3) > 1
                    if map(ur + moves(i,1), uc) == 1 || map(ur, uc + moves(i,2)) == 1
                        continue;
                    end
                end
                
                new_rhs = g_val(ur, uc) + moves(i, 3);
                if new_rhs < rhs(nr, nc)
                    rhs(nr, nc) = new_rhs;
                end
                
                update_vertex(nr, nc);
            end
            
        else
            % Nodo localmente subestimado -> invalidar
            g_val(ur, uc) = inf;
            
            % Actualizar el nodo mismo y sus predecesores
            nodes_to_update = [ur, uc];
            for i = 1:size(moves, 1)
                nr = ur + moves(i, 1);
                nc = uc + moves(i, 2);
                if nr >= 1 && nr <= rows && nc >= 1 && nc <= cols
                    nodes_to_update = [nodes_to_update; nr, nc];
                end
            end
            
            for j = 1:size(nodes_to_update, 1)
                nr = nodes_to_update(j, 1);
                nc = nodes_to_update(j, 2);
                
                if map(nr, nc) == 1
                    continue;
                end
                
                if nr == goal(1) && nc == goal(2)
                    continue;  % La meta siempre tiene rhs = 0
                end
                
                % Recalcular rhs como minimo de vecinos
                min_rhs = inf;
                for k = 1:size(moves, 1)
                    vr = nr + moves(k, 1);
                    vc = nc + moves(k, 2);
                    if vr >= 1 && vr <= rows && vc >= 1 && vc <= cols
                        if map(vr, vc) == 0
                            if moves(k, 3) > 1
                                if map(nr + moves(k,1), nc) == 1 || ...
                                        map(nr, nc + moves(k,2)) == 1
                                    continue;
                                end
                            end
                            candidate = g_val(vr, vc) + moves(k, 3);
                            if candidate < min_rhs
                                min_rhs = candidate;
                            end
                        end
                    end
                end
                rhs(nr, nc) = min_rhs;
                
                update_vertex(nr, nc);
            end
        end
    end
    
    % --- Paso 5: Reconstruir ruta desde start hasta goal ---
    if g_val(start(1), start(2)) == inf
        fprintf('      [D* Lite] No se encontro ruta valida.\n');
        path = [];
        total_cost = inf;
        return;
    end
    
    path = start;
    current = start;
    total_cost = 0;
    max_path_len = rows * cols;
    
    while ~(current(1) == goal(1) && current(2) == goal(2))
        % Buscar el vecino con menor g + costo de movimiento
        best_g = inf;
        best_next = current;
        best_cost = 0;
        
        for i = 1:size(moves, 1)
            nr = current(1) + moves(i, 1);
            nc = current(2) + moves(i, 2);
            
            if nr < 1 || nr > rows || nc < 1 || nc > cols
                continue;
            end
            if map(nr, nc) == 1
                continue;
            end
            
            % Verificar diagonales
            if moves(i, 3) > 1
                if map(current(1) + moves(i,1), current(2)) == 1 || ...
                        map(current(1), current(2) + moves(i,2)) == 1
                    continue;
                end
            end
            
            candidate = g_val(nr, nc) + moves(i, 3);
            if g_val(nr, nc) < best_g
                best_g = g_val(nr, nc);
                best_next = [nr, nc];
                best_cost = moves(i, 3);
            end
        end
        
        if best_next(1) == current(1) && best_next(2) == current(2)
            fprintf('      [D* Lite] Ruta bloqueada durante reconstruccion.\n');
            path = [];
            total_cost = inf;
            return;
        end
        
        current = best_next;
        path = [path; current];
        total_cost = total_cost + best_cost;
        
        if size(path, 1) > max_path_len
            warning('Reconstruccion de ruta excedio limite');
            break;
        end
    end
    
    fprintf('      [D* Lite] Replanificacion exitosa.\n');
    
    % --- Funcion anidada: update_vertex ---
    function update_vertex(nr, nc)
        if g_val(nr, nc) ~= rhs(nr, nc)
            % Nodo inconsistente -> agregar/actualizar en cola
            key_n = calculate_key([nr, nc], start, g_val, rhs, km);
            if in_queue(nr, nc)
                % Actualizar key (eliminar y reinsertar)
                idx = find(priority_queue(:,1) == nr & priority_queue(:,2) == nc, 1);
                if ~isempty(idx)
                    priority_queue(idx, :) = [nr, nc, key_n(1), key_n(2)];
                end
            else
                priority_queue = [priority_queue; nr, nc, key_n(1), key_n(2)];
                in_queue(nr, nc) = true;
            end
        else
            % Nodo consistente -> sacar de cola si esta
            if in_queue(nr, nc)
                idx = find(priority_queue(:,1) == nr & priority_queue(:,2) == nc, 1);
                if ~isempty(idx)
                    priority_queue(idx, :) = [];
                end
                in_queue(nr, nc) = false;
            end
        end
    end
end

%% ========================================================================
%  FUNCIONES AUXILIARES
% =========================================================================

function key = calculate_key(s, start, g_val, rhs, km)
% Calcula la key de prioridad para un nodo en D* Lite
    g_s = g_val(s(1), s(2));
    rhs_s = rhs(s(1), s(2));
    min_g_rhs = min(g_s, rhs_s);
    h = sqrt((s(1) - start(1))^2 + (s(2) - start(2))^2);
    key = [min_g_rhs + h + km, min_g_rhs];
end

function result = compare_keys(k1, k2)
% Compara dos keys de D* Lite
% Retorna: -1 si k1 < k2, 0 si iguales, 1 si k1 > k2
    if k1(1) < k2(1)
        result = -1;
    elseif k1(1) > k2(1)
        result = 1;
    elseif k1(2) < k2(2)
        result = -1;
    elseif k1(2) > k2(2)
        result = 1;
    else
        result = 0;
    end
end

function free_cell = find_free_cell(map, pos)
% Busca la celda libre mas cercana a una posicion dada
    [rows, cols] = size(map);
    for radius = 1:50
        for dr = -radius:radius
            for dc = -radius:radius
                nr = pos(1) + dr;
                nc = pos(2) + dc;
                if nr >= 1 && nr <= rows && nc >= 1 && nc <= cols
                    if map(nr, nc) == 0
                        free_cell = [nr, nc];
                        return;
                    end
                end
            end
        end
    end
    free_cell = [];
end
