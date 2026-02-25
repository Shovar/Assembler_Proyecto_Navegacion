function plot_replanning(map, old_path, new_path, robot_pos, goal, ...
    obs_center, obs_size, titulo)
% PLOT_REPLANNING - Visualiza el escenario de replanificacion

    imagesc(map);
    colormap(flipud(gray));
    hold on;
    
    % Ruta original (en gris discontinuo - ya no es valida)
    h1 = plot(old_path(:,2), old_path(:,1), '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5);
    
    % Nueva ruta (en rojo)
    h2 = plot(new_path(:,2), new_path(:,1), 'r-', 'LineWidth', 2.5);
    
    % Obstaculo dinamico (rectangulo naranja + marcador para leyenda)
    half = floor(obs_size / 2);
    rect_x = obs_center(2) - half;
    rect_y = obs_center(1) - half;
    rectangle('Position', [rect_x, rect_y, obs_size, obs_size], ...
        'EdgeColor', [1 0.5 0], 'LineWidth', 2.5, 'LineStyle', '-');
    h3 = plot(obs_center(2), obs_center(1), 's', 'Color', [1 0.5 0], ...
        'MarkerSize', 10, 'MarkerFaceColor', [1 0.5 0], 'LineWidth', 2);
    
    % Posicion actual del robot
    h4 = plot(robot_pos(2), robot_pos(1), 'g^', 'MarkerSize', 14, ...
        'MarkerFaceColor', 'g', 'LineWidth', 2);
    
    % Meta
    h5 = plot(goal(2), goal(1), 'bp', 'MarkerSize', 14, ...
        'MarkerFaceColor', 'b', 'LineWidth', 2);
    
    legend([h1 h2 h3 h4 h5], 'Ruta original (A*)', 'Nueva ruta (D* Lite)', ...
        'Obstaculo dinamico', 'Robot', 'Meta', 'Location', 'best');
    title(titulo, 'FontSize', 14);
    xlabel('Columna (celdas)');
    ylabel('Fila (celdas)');
    axis equal tight;
    grid on;
    set(gca, 'FontSize', 11);
    hold off;
end
