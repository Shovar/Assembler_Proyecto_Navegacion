function plot_map_with_path(map, path, start, goal, titulo)
% PLOT_MAP_WITH_PATH - Visualiza un mapa con la ruta planificada

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
