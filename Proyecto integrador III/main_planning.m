%% MAIN_PLANNING.M - Sistema de Planificacion de Trayectorias
% Universidad Europea - Navegacion Autonoma de Robots
% Proyecto Integrador (III) - Planificacion de Trayectorias
%
% Este script implementa un sistema de planificacion (A*) y replanificacion
% (D* Lite) sobre mapas 2D de ocupacion. Se trabaja con:
%   1. El mapa generado por SLAM (Proyecto Integrador II)
%   2. Un mapa alternativo disenado por el equipo
%
% Conexion con proyectos anteriores:
%   - Proyecto I (Localizacion): EKF Odom+IMU -> Poses estimadas
%   - Proyecto II (SLAM): Occupancy Grid Mapping -> Mapa 2D
%   - Proyecto III (este): A* + D* Lite -> Ruta navegable con replanificacion
%
% Autores: Antonio Garcia Alcon, Adrian Santero Alonso, Johalex Jose Arrieta Perez
% Fecha: Febrero 2026

clc; clear; close all;

fprintf('=============================================================\n');
fprintf('   PROYECTO INTEGRADOR (III): PLANIFICACION DE TRAYECTORIAS\n');
fprintf('   Robot MiR100 - A* + D* Lite\n');
fprintf('=============================================================\n\n');

%% ========================================================================
%  PASO 1: CARGAR MAPAS
% =========================================================================
fprintf('PASO 1: Cargando mapas...\n');
fprintf('---------------------------------------------------------\n');

% --- Mapa 1: Mapa del SLAM (Proyecto II) ---
% Si tienes el mapa guardado del proyecto anterior, cargalo aqui.
% Si no, usamos una version simulada basada en el mapa de referencia del rosbag.
[map_slam, map_slam_info] = load_slam_map();
fprintf('  Mapa SLAM: %dx%d celdas (%.2f m/celda)\n', ...
    size(map_slam,1), size(map_slam,2), map_slam_info.resolution);

% --- Mapa 2: Mapa alternativo ---
[map_alt, map_alt_info] = create_alternative_map();
fprintf('  Mapa alternativo: %dx%d celdas (%.2f m/celda)\n', ...
    size(map_alt,1), size(map_alt,2), map_alt_info.resolution);

%% ========================================================================
%  PASO 2: DEFINIR PUNTOS DE INICIO Y META
% =========================================================================
fprintf('\nPASO 2: Definiendo puntos de inicio y meta...\n');
fprintf('---------------------------------------------------------\n');

% --- Puntos para el mapa SLAM ---
% Estos puntos se eligen en zonas libres del mapa del MiR100
start_slam = [50, 30];    % Punto de inicio [fila, columna]
goal_slam  = [180, 350];  % Punto de meta

% Verificar que los puntos estan en zona libre
if map_slam(start_slam(1), start_slam(2)) == 1
    warning('Inicio SLAM esta en obstaculo. Ajustando...');
    start_slam = find_free_cell(map_slam, start_slam);
end
if map_slam(goal_slam(1), goal_slam(2)) == 1
    warning('Meta SLAM esta en obstaculo. Ajustando...');
    goal_slam = find_free_cell(map_slam, goal_slam);
end

fprintf('  SLAM - Inicio: [%d, %d], Meta: [%d, %d]\n', ...
    start_slam(1), start_slam(2), goal_slam(1), goal_slam(2));

% --- Puntos para el mapa alternativo ---
start_alt = [20, 20];
goal_alt  = [180, 180];

if map_alt(start_alt(1), start_alt(2)) == 1
    start_alt = find_free_cell(map_alt, start_alt);
end
if map_alt(goal_alt(1), goal_alt(2)) == 1
    goal_alt = find_free_cell(map_alt, goal_alt);
end

fprintf('  Alternativo - Inicio: [%d, %d], Meta: [%d, %d]\n', ...
    start_alt(1), start_alt(2), goal_alt(1), goal_alt(2));

%% ========================================================================
%  PASO 3: PLANIFICACION INICIAL CON A*
% =========================================================================
fprintf('\nPASO 3: Planificacion inicial con A*...\n');
fprintf('---------------------------------------------------------\n');

% --- A* sobre mapa SLAM ---
fprintf('  Ejecutando A* en mapa SLAM...\n');
tic;
[path_slam, nodes_expanded_slam, cost_slam] = astar(map_slam, start_slam, goal_slam);
time_astar_slam = toc;

if isempty(path_slam)
    fprintf('  [!] No se encontro ruta en mapa SLAM.\n');
else
    fprintf('  Ruta encontrada: %d celdas, costo: %.2f, nodos expandidos: %d\n', ...
        size(path_slam,1), cost_slam, nodes_expanded_slam);
    fprintf('  Tiempo de computo: %.4f s\n', time_astar_slam);
end

% --- A* sobre mapa alternativo ---
fprintf('  Ejecutando A* en mapa alternativo...\n');
tic;
[path_alt, nodes_expanded_alt, cost_alt] = astar(map_alt, start_alt, goal_alt);
time_astar_alt = toc;

if isempty(path_alt)
    fprintf('  [!] No se encontro ruta en mapa alternativo.\n');
else
    fprintf('  Ruta encontrada: %d celdas, costo: %.2f, nodos expandidos: %d\n', ...
        size(path_alt,1), cost_alt, nodes_expanded_alt);
    fprintf('  Tiempo de computo: %.4f s\n', time_astar_alt);
end

%% ========================================================================
%  PASO 4: SIMULACION DE OBSTACULOS DINAMICOS
% =========================================================================
fprintf('\nPASO 4: Simulando obstaculos dinamicos...\n');
fprintf('---------------------------------------------------------\n');

% --- Escenario A: Obstaculo parcial (bloquea parte de la ruta) ---
fprintf('  Escenario A: Obstaculo parcial en ruta...\n');

% Encontrar un punto medio de la ruta para colocar el obstaculo
if ~isempty(path_slam)
    idx_mid = round(size(path_slam,1) * 0.4);  % 40% del camino
    obs_center_slam_A = path_slam(idx_mid, :);
    obs_size_A = 8;  % Tamanio del obstaculo (8x8 celdas)
    
    map_slam_A = add_dynamic_obstacle(map_slam, obs_center_slam_A, obs_size_A);
    fprintf('    Obstaculo de %dx%d en [%d, %d]\n', ...
        obs_size_A, obs_size_A, obs_center_slam_A(1), obs_center_slam_A(2));
end

if ~isempty(path_alt)
    idx_mid_alt = round(size(path_alt,1) * 0.4);
    obs_center_alt_A = path_alt(idx_mid_alt, :);
    
    map_alt_A = add_dynamic_obstacle(map_alt, obs_center_alt_A, obs_size_A);
    fprintf('    Obstaculo de %dx%d en [%d, %d] (mapa alt.)\n', ...
        obs_size_A, obs_size_A, obs_center_alt_A(1), obs_center_alt_A(2));
end

% --- Escenario B: Obstaculo grande (bloquea totalmente un pasillo) ---
fprintf('  Escenario B: Obstaculo grande bloqueando ruta...\n');

if ~isempty(path_slam)
    idx_mid2 = round(size(path_slam,1) * 0.6);  % 60% del camino
    obs_center_slam_B = path_slam(idx_mid2, :);
    obs_size_B = 15;  % Obstaculo mas grande
    
    map_slam_B = add_dynamic_obstacle(map_slam, obs_center_slam_B, obs_size_B);
    fprintf('    Obstaculo de %dx%d en [%d, %d]\n', ...
        obs_size_B, obs_size_B, obs_center_slam_B(1), obs_center_slam_B(2));
end

if ~isempty(path_alt)
    idx_mid2_alt = round(size(path_alt,1) * 0.6);
    obs_center_alt_B = path_alt(idx_mid2_alt, :);
    
    map_alt_B = add_dynamic_obstacle(map_alt, obs_center_alt_B, obs_size_B);
    fprintf('    Obstaculo de %dx%d en [%d, %d] (mapa alt.)\n', ...
        obs_size_B, obs_size_B, obs_center_alt_B(1), obs_center_alt_B(2));
end

%% ========================================================================
%  PASO 5: REPLANIFICACION CON D* LITE
% =========================================================================
fprintf('\nPASO 5: Replanificacion con D* Lite...\n');
fprintf('---------------------------------------------------------\n');

% --- Escenario A: Replanificacion tras obstaculo parcial ---
fprintf('  [Escenario A - SLAM] Replanificando...\n');
if ~isempty(path_slam)
    % El robot esta en el punto donde detecta el obstaculo
    robot_pos_A = path_slam(max(1, idx_mid - 10), :);  % Un poco antes del obstaculo
    
    tic;
    [replan_path_slam_A, replan_nodes_A, replan_cost_A] = dstar_lite(...
        map_slam_A, robot_pos_A, goal_slam, path_slam);
    time_replan_slam_A = toc;
    
    if ~isempty(replan_path_slam_A)
        fprintf('    Nueva ruta: %d celdas, costo: %.2f, nodos: %d\n', ...
            size(replan_path_slam_A,1), replan_cost_A, replan_nodes_A);
        fprintf('    Tiempo replanificacion: %.4f s\n', time_replan_slam_A);
    else
        fprintf('    [!] No se encontro ruta alternativa.\n');
    end
end

fprintf('  [Escenario A - Alternativo] Replanificando...\n');
if ~isempty(path_alt)
    robot_pos_alt_A = path_alt(max(1, idx_mid_alt - 10), :);
    
    tic;
    [replan_path_alt_A, replan_nodes_alt_A, replan_cost_alt_A] = dstar_lite(...
        map_alt_A, robot_pos_alt_A, goal_alt, path_alt);
    time_replan_alt_A = toc;
    
    if ~isempty(replan_path_alt_A)
        fprintf('    Nueva ruta: %d celdas, costo: %.2f, nodos: %d\n', ...
            size(replan_path_alt_A,1), replan_cost_alt_A, replan_nodes_alt_A);
        fprintf('    Tiempo replanificacion: %.4f s\n', time_replan_alt_A);
    else
        fprintf('    [!] No se encontro ruta alternativa.\n');
    end
end

% --- Escenario B: Replanificacion tras obstaculo grande ---
fprintf('  [Escenario B - SLAM] Replanificando...\n');
if ~isempty(path_slam)
    robot_pos_B = path_slam(max(1, idx_mid2 - 15), :);
    
    tic;
    [replan_path_slam_B, replan_nodes_B, replan_cost_B] = dstar_lite(...
        map_slam_B, robot_pos_B, goal_slam, path_slam);
    time_replan_slam_B = toc;
    
    if ~isempty(replan_path_slam_B)
        fprintf('    Nueva ruta: %d celdas, costo: %.2f, nodos: %d\n', ...
            size(replan_path_slam_B,1), replan_cost_B, replan_nodes_B);
        fprintf('    Tiempo replanificacion: %.4f s\n', time_replan_slam_B);
    else
        fprintf('    [!] No se encontro ruta alternativa.\n');
    end
end

fprintf('  [Escenario B - Alternativo] Replanificando...\n');
if ~isempty(path_alt)
    robot_pos_alt_B = path_alt(max(1, idx_mid2_alt - 15), :);
    
    tic;
    [replan_path_alt_B, replan_nodes_alt_B, replan_cost_alt_B] = dstar_lite(...
        map_alt_B, robot_pos_alt_B, goal_alt, path_alt);
    time_replan_alt_B = toc;
    
    if ~isempty(replan_path_alt_B)
        fprintf('    Nueva ruta: %d celdas, costo: %.2f, nodos: %d\n', ...
            size(replan_path_alt_B,1), replan_cost_alt_B, replan_nodes_alt_B);
        fprintf('    Tiempo replanificacion: %.4f s\n', time_replan_alt_B);
    else
        fprintf('    [!] No se encontro ruta alternativa.\n');
    end
end

%% ========================================================================
%  PASO 6: EVALUACION DE METRICAS
% =========================================================================
fprintf('\nPASO 6: Evaluando metricas de calidad...\n');
fprintf('---------------------------------------------------------\n');

% Evaluar todas las rutas
metrics = struct();

if ~isempty(path_slam)
    metrics.slam_original = evaluate_path(path_slam, map_slam, time_astar_slam, nodes_expanded_slam);
    fprintf('  SLAM Original  -> Longitud: %.2f m, Suavidad: %.4f, Clearance: %.2f\n', ...
        metrics.slam_original.length_m, metrics.slam_original.smoothness, metrics.slam_original.avg_clearance);
end

if ~isempty(path_slam) && ~isempty(replan_path_slam_A)
    metrics.slam_replan_A = evaluate_path(replan_path_slam_A, map_slam_A, time_replan_slam_A, replan_nodes_A);
    fprintf('  SLAM Replan A  -> Longitud: %.2f m, Suavidad: %.4f, Clearance: %.2f\n', ...
        metrics.slam_replan_A.length_m, metrics.slam_replan_A.smoothness, metrics.slam_replan_A.avg_clearance);
end

if ~isempty(path_slam) && ~isempty(replan_path_slam_B)
    metrics.slam_replan_B = evaluate_path(replan_path_slam_B, map_slam_B, time_replan_slam_B, replan_nodes_B);
    fprintf('  SLAM Replan B  -> Longitud: %.2f m, Suavidad: %.4f, Clearance: %.2f\n', ...
        metrics.slam_replan_B.length_m, metrics.slam_replan_B.smoothness, metrics.slam_replan_B.avg_clearance);
end

if ~isempty(path_alt)
    metrics.alt_original = evaluate_path(path_alt, map_alt, time_astar_alt, nodes_expanded_alt);
    fprintf('  Alt. Original  -> Longitud: %.2f m, Suavidad: %.4f, Clearance: %.2f\n', ...
        metrics.alt_original.length_m, metrics.alt_original.smoothness, metrics.alt_original.avg_clearance);
end

if ~isempty(path_alt) && ~isempty(replan_path_alt_A)
    metrics.alt_replan_A = evaluate_path(replan_path_alt_A, map_alt_A, time_replan_alt_A, replan_nodes_alt_A);
    fprintf('  Alt. Replan A  -> Longitud: %.2f m, Suavidad: %.4f, Clearance: %.2f\n', ...
        metrics.alt_replan_A.length_m, metrics.alt_replan_A.smoothness, metrics.alt_replan_A.avg_clearance);
end

if ~isempty(path_alt) && ~isempty(replan_path_alt_B)
    metrics.alt_replan_B = evaluate_path(replan_path_alt_B, map_alt_B, time_replan_alt_B, replan_nodes_alt_B);
    fprintf('  Alt. Replan B  -> Longitud: %.2f m, Suavidad: %.4f, Clearance: %.2f\n', ...
        metrics.alt_replan_B.length_m, metrics.alt_replan_B.smoothness, metrics.alt_replan_B.avg_clearance);
end

%% ========================================================================
%  PASO 7: GENERAR FIGURAS
% =========================================================================
fprintf('\nPASO 7: Generando figuras para el informe...\n');
fprintf('---------------------------------------------------------\n');

% --- Figura 1: Mapa SLAM con ruta inicial ---
figure('Name', 'Ruta Inicial - Mapa SLAM', 'Position', [100 100 800 600]);
plot_map_with_path(map_slam, path_slam, start_slam, goal_slam, ...
    'Ruta Inicial A* - Mapa SLAM (Proyecto II)');
saveas(gcf, 'fig_slam_ruta_inicial.png');

% --- Figura 2: Mapa alternativo con ruta inicial ---
figure('Name', 'Ruta Inicial - Mapa Alternativo', 'Position', [150 100 800 600]);
plot_map_with_path(map_alt, path_alt, start_alt, goal_alt, ...
    'Ruta Inicial A* - Mapa Alternativo');
saveas(gcf, 'fig_alt_ruta_inicial.png');

% --- Figura 3: Replanificacion Escenario A - SLAM ---
if ~isempty(path_slam) && ~isempty(replan_path_slam_A)
    figure('Name', 'Replanificacion A - SLAM', 'Position', [200 100 800 600]);
    plot_replanning(map_slam_A, path_slam, replan_path_slam_A, ...
        robot_pos_A, goal_slam, obs_center_slam_A, obs_size_A, ...
        'Replanificacion D* Lite (Obst. Parcial) - Mapa SLAM');
    saveas(gcf, 'fig_slam_replan_A.png');
end

% --- Figura 4: Replanificacion Escenario B - SLAM ---
if ~isempty(path_slam) && ~isempty(replan_path_slam_B)
    figure('Name', 'Replanificacion B - SLAM', 'Position', [250 100 800 600]);
    plot_replanning(map_slam_B, path_slam, replan_path_slam_B, ...
        robot_pos_B, goal_slam, obs_center_slam_B, obs_size_B, ...
        'Replanificacion D* Lite (Obst. Grande) - Mapa SLAM');
    saveas(gcf, 'fig_slam_replan_B.png');
end

% --- Figura 5: Replanificacion Escenario A - Alternativo ---
if ~isempty(path_alt) && ~isempty(replan_path_alt_A)
    figure('Name', 'Replanificacion A - Alternativo', 'Position', [300 100 800 600]);
    plot_replanning(map_alt_A, path_alt, replan_path_alt_A, ...
        robot_pos_alt_A, goal_alt, obs_center_alt_A, obs_size_A, ...
        'Replanificacion D* Lite (Obst. Parcial) - Mapa Alternativo');
    saveas(gcf, 'fig_alt_replan_A.png');
end

% --- Figura 6: Replanificacion Escenario B - Alternativo ---
if ~isempty(path_alt) && ~isempty(replan_path_alt_B)
    figure('Name', 'Replanificacion B - Alternativo', 'Position', [350 100 800 600]);
    plot_replanning(map_alt_B, path_alt, replan_path_alt_B, ...
        robot_pos_alt_B, goal_alt, obs_center_alt_B, obs_size_B, ...
        'Replanificacion D* Lite (Obst. Grande) - Mapa Alternativo');
    saveas(gcf, 'fig_alt_replan_B.png');
end

% --- Figura 7: Comparativa de metricas ---
figure('Name', 'Comparativa de Metricas', 'Position', [400 100 1000 500]);
plot_metrics_comparison(metrics);
saveas(gcf, 'fig_metricas_comparativa.png');

%% ========================================================================
%  PASO 8: TABLA RESUMEN
% =========================================================================
fprintf('\nPASO 8: Tabla resumen de resultados\n');
fprintf('=========================================================\n');
fprintf('%-25s | %-10s | %-10s | %-10s | %-12s | %-10s\n', ...
    'Escenario', 'Celdas', 'Long.(m)', 'Suavidad', 'T. comp.(s)', 'Nodos');
fprintf('---------------------------------------------------------');
fprintf('--------------------------------------\n');

if isfield(metrics, 'slam_original')
    m = metrics.slam_original;
    fprintf('%-25s | %-10d | %-10.2f | %-10.4f | %-12.4f | %-10d\n', ...
        'SLAM Original', m.num_cells, m.length_m, m.smoothness, m.comp_time, m.nodes_expanded);
end
if isfield(metrics, 'slam_replan_A')
    m = metrics.slam_replan_A;
    fprintf('%-25s | %-10d | %-10.2f | %-10.4f | %-12.4f | %-10d\n', ...
        'SLAM Replan (parcial)', m.num_cells, m.length_m, m.smoothness, m.comp_time, m.nodes_expanded);
end
if isfield(metrics, 'slam_replan_B')
    m = metrics.slam_replan_B;
    fprintf('%-25s | %-10d | %-10.2f | %-10.4f | %-12.4f | %-10d\n', ...
        'SLAM Replan (grande)', m.num_cells, m.length_m, m.smoothness, m.comp_time, m.nodes_expanded);
end
if isfield(metrics, 'alt_original')
    m = metrics.alt_original;
    fprintf('%-25s | %-10d | %-10.2f | %-10.4f | %-12.4f | %-10d\n', ...
        'Alt. Original', m.num_cells, m.length_m, m.smoothness, m.comp_time, m.nodes_expanded);
end
if isfield(metrics, 'alt_replan_A')
    m = metrics.alt_replan_A;
    fprintf('%-25s | %-10d | %-10.2f | %-10.4f | %-12.4f | %-10d\n', ...
        'Alt. Replan (parcial)', m.num_cells, m.length_m, m.smoothness, m.comp_time, m.nodes_expanded);
end
if isfield(metrics, 'alt_replan_B')
    m = metrics.alt_replan_B;
    fprintf('%-25s | %-10d | %-10.2f | %-10.4f | %-12.4f | %-10d\n', ...
        'Alt. Replan (grande)', m.num_cells, m.length_m, m.smoothness, m.comp_time, m.nodes_expanded);
end


