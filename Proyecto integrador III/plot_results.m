function plot_map_with_path(map, path, start, goal, titulo)
% PLOT_MAP_WITH_PATH - Visualiza un mapa con la ruta planificada
%
% Entradas:
%   map    : Mapa binario
%   path   : Ruta Nx2
%   start  : Punto de inicio
%   goal   : Punto de meta
%   titulo : Titulo de la figura

    imagesc(map);
    colormap(flipud(gray));
    hold on;
    
    if ~isempty(path)
        plot(path(:,2), path(:,1), 'r-', 'LineWidth', 2);
    end
    
    plot(start(2), start(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2);
    plot(goal(2), goal(1), 'bp', 'MarkerSize', 14, 'MarkerFaceColor', 'b', 'LineWidth', 2);
    
    legend('Ruta A*', 'Inicio', 'Meta', 'Location', 'best');
    title(titulo, 'FontSize', 14);
    xlabel('Columna (celdas)');
    ylabel('Fila (celdas)');
    axis equal tight;
    grid on;
    set(gca, 'FontSize', 11);
    hold off;
end

function plot_replanning(map, old_path, new_path, robot_pos, goal, ...
    obs_center, obs_size, titulo)
% PLOT_REPLANNING - Visualiza el escenario de replanificacion
%
% Muestra la ruta original (invalidada), el obstaculo dinamico y la
% nueva ruta calculada por D* Lite.

    imagesc(map);
    colormap(flipud(gray));
    hold on;
    
    % Ruta original (en gris discontinuo - ya no es valida)
    if ~isempty(old_path)
        plot(old_path(:,2), old_path(:,1), '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5);
    end
    
    % Nueva ruta (en rojo)
    if ~isempty(new_path)
        plot(new_path(:,2), new_path(:,1), 'r-', 'LineWidth', 2.5);
    end
    
    % Obstaculo dinamico (rectangulo naranja semitransparente)
    half = floor(obs_size / 2);
    rect_x = obs_center(2) - half;
    rect_y = obs_center(1) - half;
    rectangle('Position', [rect_x, rect_y, obs_size, obs_size], ...
        'EdgeColor', [1 0.5 0], 'LineWidth', 2.5, 'LineStyle', '-');
    
    % Posicion actual del robot
    plot(robot_pos(2), robot_pos(1), 'g^', 'MarkerSize', 14, ...
        'MarkerFaceColor', 'g', 'LineWidth', 2);
    
    % Meta
    plot(goal(2), goal(1), 'bp', 'MarkerSize', 14, ...
        'MarkerFaceColor', 'b', 'LineWidth', 2);
    
    legend('Ruta original (A*)', 'Nueva ruta (D* Lite)', ...
        'Obstaculo dinamico', 'Robot', 'Meta', 'Location', 'best');
    title(titulo, 'FontSize', 14);
    xlabel('Columna (celdas)');
    ylabel('Fila (celdas)');
    axis equal tight;
    grid on;
    set(gca, 'FontSize', 11);
    hold off;
end

function plot_metrics_comparison(metrics)
% PLOT_METRICS_COMPARISON - Grafica comparativa de metricas
%
% Genera subplots con las metricas de todas las rutas para comparacion

    % Recopilar datos
    names = {};
    lengths = [];
    smooths = [];
    clearances = [];
    times = [];
    nodes = [];
    
    fields = {'slam_original', 'slam_replan_A', 'slam_replan_B', ...
              'alt_original', 'alt_replan_A', 'alt_replan_B'};
    labels = {'SLAM\nOriginal', 'SLAM\nReplan A', 'SLAM\nReplan B', ...
              'Alt.\nOriginal', 'Alt.\nReplan A', 'Alt.\nReplan B'};
    
    for i = 1:length(fields)
        if isfield(metrics, fields{i})
            m = metrics.(fields{i});
            names{end+1} = labels{i};
            lengths(end+1) = m.length_m;
            smooths(end+1) = m.smoothness;
            clearances(end+1) = m.avg_clearance;
            times(end+1) = m.comp_time;
            nodes(end+1) = m.nodes_expanded;
        end
    end
    
    n = length(names);
    if n == 0
        text(0.5, 0.5, 'No hay datos de metricas', 'HorizontalAlignment', 'center');
        return;
    end
    
    colors_slam = [0.2 0.4 0.8;  0.4 0.6 0.9;  0.6 0.75 0.95];
    colors_alt  = [0.8 0.2 0.2;  0.9 0.4 0.4;  0.95 0.6 0.6];
    bar_colors = [];
    for i = 1:n
        if i <= 3
            bar_colors = [bar_colors; colors_slam(min(i,3), :)];
        else
            bar_colors = [bar_colors; colors_alt(min(i-3,3), :)];
        end
    end
    
    x = 1:n;
    
    % Subplot 1: Longitud
    subplot(2, 3, 1);
    b = bar(x, lengths, 0.7);
    b.FaceColor = 'flat';
    for i = 1:n, b.CData(i,:) = bar_colors(i,:); end
    set(gca, 'XTick', x, 'XTickLabel', names, 'FontSize', 9);
    ylabel('Longitud (m)');
    title('Longitud de Ruta', 'FontSize', 11);
    grid on;
    
    % Subplot 2: Suavidad
    subplot(2, 3, 2);
    b = bar(x, smooths, 0.7);
    b.FaceColor = 'flat';
    for i = 1:n, b.CData(i,:) = bar_colors(i,:); end
    set(gca, 'XTick', x, 'XTickLabel', names, 'FontSize', 9);
    ylabel('Suavidad (rad)');
    title('Suavidad (menor = mejor)', 'FontSize', 11);
    grid on;
    
    % Subplot 3: Clearance
    subplot(2, 3, 3);
    b = bar(x, clearances, 0.7);
    b.FaceColor = 'flat';
    for i = 1:n, b.CData(i,:) = bar_colors(i,:); end
    set(gca, 'XTick', x, 'XTickLabel', names, 'FontSize', 9);
    ylabel('Clearance (celdas)');
    title('Distancia a Obstaculos', 'FontSize', 11);
    grid on;
    
    % Subplot 4: Tiempo de computo
    subplot(2, 3, 4);
    b = bar(x, times * 1000, 0.7);  % Convertir a ms
    b.FaceColor = 'flat';
    for i = 1:n, b.CData(i,:) = bar_colors(i,:); end
    set(gca, 'XTick', x, 'XTickLabel', names, 'FontSize', 9);
    ylabel('Tiempo (ms)');
    title('Tiempo de Computo', 'FontSize', 11);
    grid on;
    
    % Subplot 5: Nodos expandidos
    subplot(2, 3, 5);
    b = bar(x, nodes, 0.7);
    b.FaceColor = 'flat';
    for i = 1:n, b.CData(i,:) = bar_colors(i,:); end
    set(gca, 'XTick', x, 'XTickLabel', names, 'FontSize', 9);
    ylabel('Nodos');
    title('Nodos Expandidos', 'FontSize', 11);
    grid on;
    
    % Subplot 6: Leyenda
    subplot(2, 3, 6);
    axis off;
    text(0.5, 0.8, 'Leyenda:', 'FontSize', 12, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center');
    text(0.5, 0.6, 'Azules = Mapa SLAM', 'FontSize', 11, 'Color', [0.2 0.4 0.8], ...
        'HorizontalAlignment', 'center');
    text(0.5, 0.4, 'Rojos = Mapa Alternativo', 'FontSize', 11, 'Color', [0.8 0.2 0.2], ...
        'HorizontalAlignment', 'center');
    text(0.5, 0.15, {'Original = A*', 'Replan A = Obst. parcial', 'Replan B = Obst. grande'}, ...
        'FontSize', 10, 'HorizontalAlignment', 'center');
    
    sgtitle('Comparativa de Metricas - Planificacion y Replanificacion', ...
        'FontSize', 14, 'FontWeight', 'bold');
end
