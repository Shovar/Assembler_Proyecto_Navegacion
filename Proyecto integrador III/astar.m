function [path, nodes_expanded, total_cost] = astar(map, start, goal)
% ASTAR - Algoritmo A* para planificacion de trayectorias en grid 2D
%
% Implementa el algoritmo A* (Hart, Nilsson & Raphael, 1968) para encontrar
% la ruta optima entre dos puntos en un mapa de ocupacion.
%
% Entradas:
%   map   : Matriz binaria (0=libre, 1=ocupado)
%   start : [fila, columna] del punto de inicio
%   goal  : [fila, columna] del punto de meta
%
% Salidas:
%   path           : Matriz Nx2 con las coordenadas de la ruta [fila, col]
%   nodes_expanded : Numero de nodos expandidos durante la busqueda
%   total_cost     : Costo total de la ruta encontrada
%
% El algoritmo utiliza:
%   - Conectividad 8 (8 vecinos: 4 cardinales + 4 diagonales)
%   - Heuristica: Distancia euclidiana (admisible y consistente)
%   - Costo de movimiento: 1 para cardinales, sqrt(2) para diagonales
%
% Referencias:
%   Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A Formal Basis for 
%   the Heuristic Determination of Minimum Cost Paths. IEEE Transactions on 
%   Systems Science and Cybernetics, 4(2), 100-107.
%
% Autores: Antonio Garcia Alcon, Adrian Santero Alonso, Johalex Jose Arrieta Perez

    [rows, cols] = size(map);
    
    % Validaciones basicas
    if start(1) < 1 || start(1) > rows || start(2) < 1 || start(2) > cols
        error('Punto de inicio fuera del mapa');
    end
    if goal(1) < 1 || goal(1) > rows || goal(2) < 1 || goal(2) > cols
        error('Punto de meta fuera del mapa');
    end
    if map(start(1), start(2)) == 1
        error('Punto de inicio esta en un obstaculo');
    end
    if map(goal(1), goal(2)) == 1
        error('Punto de meta esta en un obstaculo');
    end
    
    % --- Definir movimientos (8-conectividad) ---
    %   [delta_fila, delta_col, costo]
    moves = [
        -1,  0, 1.0;    % Arriba
         1,  0, 1.0;    % Abajo
         0, -1, 1.0;    % Izquierda
         0,  1, 1.0;    % Derecha
        -1, -1, sqrt(2); % Diagonal superior izquierda
        -1,  1, sqrt(2); % Diagonal superior derecha
         1, -1, sqrt(2); % Diagonal inferior izquierda
         1,  1, sqrt(2); % Diagonal inferior derecha
    ];
    
    % --- Inicializacion de estructuras ---
    % g(n): costo real desde el inicio hasta n
    g = inf(rows, cols);
    g(start(1), start(2)) = 0;
    
    % f(n) = g(n) + h(n): costo estimado total
    f = inf(rows, cols);
    f(start(1), start(2)) = heuristic(start, goal);
    
    % Mapa de padres para reconstruir la ruta
    parent = zeros(rows, cols, 2);
    
    % Lista abierta (open list) implementada como array ordenado
    % Cada entrada: [fila, col, f_value]
    open_list = [start(1), start(2), f(start(1), start(2))];
    
    % Conjunto cerrado (closed set)
    closed = false(rows, cols);
    
    nodes_expanded = 0;
    
    % --- Bucle principal de A* ---
    while ~isempty(open_list)
        % Extraer el nodo con menor f de la lista abierta
        [~, min_idx] = min(open_list(:, 3));
        current = open_list(min_idx, 1:2);
        open_list(min_idx, :) = [];
        
        cr = current(1);
        cc = current(2);
        
        % Si ya fue visitado, saltar
        if closed(cr, cc)
            continue;
        end
        
        % Marcar como visitado
        closed(cr, cc) = true;
        nodes_expanded = nodes_expanded + 1;
        
        % Comprobar si llegamos a la meta
        if cr == goal(1) && cc == goal(2)
            % Reconstruir ruta
            path = reconstruct_path(parent, start, goal);
            total_cost = g(goal(1), goal(2));
            return;
        end
        
        % Expandir vecinos
        for i = 1:size(moves, 1)
            nr = cr + moves(i, 1);
            nc = cc + moves(i, 2);
            move_cost = moves(i, 3);
            
            % Verificar limites del mapa
            if nr < 1 || nr > rows || nc < 1 || nc > cols
                continue;
            end
            
            % Verificar que no sea obstaculo ni ya visitado
            if map(nr, nc) == 1 || closed(nr, nc)
                continue;
            end
            
            % Para movimientos diagonales, verificar que no cortemos esquinas
            % (el robot no puede pasar entre dos obstaculos en diagonal)
            if moves(i, 3) > 1  % Es diagonal
                if map(cr + moves(i,1), cc) == 1 || map(cr, cc + moves(i,2)) == 1
                    continue;
                end
            end
            
            % Calcular nuevo costo
            tentative_g = g(cr, cc) + move_cost;
            
            if tentative_g < g(nr, nc)
                % Este camino es mejor
                g(nr, nc) = tentative_g;
                f(nr, nc) = tentative_g + heuristic([nr, nc], goal);
                parent(nr, nc, :) = [cr, cc];
                
                % Agregar a la lista abierta
                open_list = [open_list; nr, nc, f(nr, nc)];
            end
        end
    end
    
    % Si llegamos aqui, no hay ruta posible
    path = [];
    total_cost = inf;
    fprintf('    [A*] No se encontro ruta hacia la meta.\n');
end

%% ========================================================================
%  FUNCIONES AUXILIARES
% =========================================================================

function h = heuristic(node, goal)
% Heuristica: Distancia euclidiana
% Es admisible (nunca sobreestima) y consistente para 8-conectividad
    h = sqrt((node(1) - goal(1))^2 + (node(2) - goal(2))^2);
end

function path = reconstruct_path(parent, start, goal)
% Reconstruye la ruta desde la meta hasta el inicio usando el mapa de padres
    path = goal;
    current = goal;
    
    max_iter = 100000;  % Seguridad contra bucles infinitos
    iter = 0;
    
    while ~(current(1) == start(1) && current(2) == start(2))
        pr = parent(current(1), current(2), 1);
        pc = parent(current(1), current(2), 2);
        
        if pr == 0 && pc == 0
            break;  % No deberia pasar si A* encontro solucion
        end
        
        current = [pr, pc];
        path = [current; path];
        
        iter = iter + 1;
        if iter > max_iter
            warning('Reconstruccion de ruta excedio el limite de iteraciones');
            break;
        end
    end
end
