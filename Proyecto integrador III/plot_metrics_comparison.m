function plot_metrics_comparison(metrics)
% PLOT_METRICS_COMPARISON - Grafica comparativa de metricas

    % Recopilar datos
    names = {};
    lengths = [];
    smooths = [];
    clearances = [];
    times = [];
    nodes = [];
    
    fields = {'slam_original', 'slam_replan_A', 'slam_replan_B', ...
              'alt_original', 'alt_replan_A', 'alt_replan_B'};
    labels = {'SLAM Original', 'SLAM Replan A', 'SLAM Replan B', ...
              'Alt. Original', 'Alt. Replan A', 'Alt. Replan B'};
    
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
    set(gca, 'XTick', x, 'XTickLabel', names, 'FontSize', 8);
    ylabel('Longitud (m)');
    title('Longitud de Ruta', 'FontSize', 11);
    grid on;
    
    % Subplot 2: Suavidad
    subplot(2, 3, 2);
    b = bar(x, smooths, 0.7);
    b.FaceColor = 'flat';
    for i = 1:n, b.CData(i,:) = bar_colors(i,:); end
    set(gca, 'XTick', x, 'XTickLabel', names, 'FontSize', 8);
    ylabel('Suavidad (rad)');
    title('Suavidad (menor = mejor)', 'FontSize', 11);
    grid on;
    
    % Subplot 3: Clearance
    subplot(2, 3, 3);
    b = bar(x, clearances, 0.7);
    b.FaceColor = 'flat';
    for i = 1:n, b.CData(i,:) = bar_colors(i,:); end
    set(gca, 'XTick', x, 'XTickLabel', names, 'FontSize', 8);
    ylabel('Clearance (celdas)');
    title('Distancia a Obstaculos', 'FontSize', 11);
    grid on;
    
    % Subplot 4: Tiempo de computo
    subplot(2, 3, 4);
    b = bar(x, times * 1000, 0.7);
    b.FaceColor = 'flat';
    for i = 1:n, b.CData(i,:) = bar_colors(i,:); end
    set(gca, 'XTick', x, 'XTickLabel', names, 'FontSize', 8);
    ylabel('Tiempo (ms)');
    title('Tiempo de Computo', 'FontSize', 11);
    grid on;
    
    % Subplot 5: Nodos expandidos
    subplot(2, 3, 5);
    b = bar(x, nodes, 0.7);
    b.FaceColor = 'flat';
    for i = 1:n, b.CData(i,:) = bar_colors(i,:); end
    set(gca, 'XTick', x, 'XTickLabel', names, 'FontSize', 8);
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
