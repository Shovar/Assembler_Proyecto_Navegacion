function metrics = evaluate_localization(est, gt, name)
% EVALUATE_LOCALIZATION Evalúa la calidad de la localización estimada

% Calcula métricas estándar:
%   - ATE: Absolute Trajectory Error (error medio de posición)
%   - RPE: Relative Pose Error (error de movimiento relativo)
%   - Error máximo, RMSE, etc.

% Uso:
%   metrics = evaluate_localization(est, gt, 'EKF')

% Entradas:
%   est  - Estructura con t, x, y, yaw estimados
%   gt   - Estructura con t, x, y, yaw ground truth
%   name - Nombre del método (para gráficas)

    if nargin < 3, name = 'Estimado'; end
    
    fprintf('\n=== EVALUACIÓN: %s ===\n', name);
    
    %% Interpolar GT a los tiempos de la estimación
    gt_x = interp1(gt.t, gt.x, est.t, 'linear', 'extrap');
    gt_y = interp1(gt.t, gt.y, est.t, 'linear', 'extrap');
    gt_yaw = interp1(gt.t, unwrap(gt.yaw), est.t, 'linear', 'extrap');
    gt_yaw = wrapToPi(gt_yaw);
    
    %% Calcular errores de posición
    err_x = est.x - gt_x;
    err_y = est.y - gt_y;
    err_pos = sqrt(err_x.^2 + err_y.^2);
    
    % Error de orientación (con wrap-around)
    err_yaw = wrapToPi(est.yaw - gt_yaw);
    
    %% ATE (Absolute Trajectory Error)
    metrics.ATE_mean = mean(err_pos);
    metrics.ATE_rmse = sqrt(mean(err_pos.^2));
    metrics.ATE_max = max(err_pos);
    metrics.ATE_std = std(err_pos);
    
    %% Error de orientación
    metrics.yaw_mean = mean(abs(err_yaw));
    metrics.yaw_rmse = sqrt(mean(err_yaw.^2));
    metrics.yaw_max = max(abs(err_yaw));
    
    %% RPE (Relative Pose Error) - cada 1 segundo
    delta_t = 1.0;  % segundos
    rpe_trans = [];
    rpe_rot = [];
    
    for i = 1:length(est.t)
        % Buscar índice delta_t segundos después
        j = find(est.t >= est.t(i) + delta_t, 1);
        if isempty(j), break; end
        
        % Movimiento estimado
        dx_est = est.x(j) - est.x(i);
        dy_est = est.y(j) - est.y(i);
        dyaw_est = wrapToPi(est.yaw(j) - est.yaw(i));
        
        % Movimiento GT
        dx_gt = gt_x(j) - gt_x(i);
        dy_gt = gt_y(j) - gt_y(i);
        dyaw_gt = wrapToPi(gt_yaw(j) - gt_yaw(i));
        
        % Error relativo
        rpe_trans = [rpe_trans; sqrt((dx_est-dx_gt)^2 + (dy_est-dy_gt)^2)];
        rpe_rot = [rpe_rot; abs(wrapToPi(dyaw_est - dyaw_gt))];
    end
    
    metrics.RPE_trans_mean = mean(rpe_trans);
    metrics.RPE_trans_rmse = sqrt(mean(rpe_trans.^2));
    metrics.RPE_rot_mean = mean(rpe_rot);
    metrics.RPE_rot_rmse = sqrt(mean(rpe_rot.^2));
    
    %% Imprimir resultados
    fprintf('\nMétricas de Trayectoria Absoluta (ATE):\n');
    fprintf('   Error medio:     %.4f m\n', metrics.ATE_mean);
    fprintf('   RMSE:            %.4f m\n', metrics.ATE_rmse);
    fprintf('   Error máximo:    %.4f m\n', metrics.ATE_max);
    fprintf('   Desv. estándar:  %.4f m\n', metrics.ATE_std);
    
    fprintf('\nMétricas de Orientación:\n');
    fprintf('   Error medio:     %.4f rad (%.2f°)\n', metrics.yaw_mean, rad2deg(metrics.yaw_mean));
    fprintf('   RMSE:            %.4f rad (%.2f°)\n', metrics.yaw_rmse, rad2deg(metrics.yaw_rmse));
    fprintf('   Error máximo:    %.4f rad (%.2f°)\n', metrics.yaw_max, rad2deg(metrics.yaw_max));
    
    fprintf('\nMétricas de Pose Relativa (RPE @ %.1fs):\n', delta_t);
    fprintf('   Traslación RMSE: %.4f m\n', metrics.RPE_trans_rmse);
    fprintf('   Rotación RMSE:   %.4f rad (%.2f°)\n', metrics.RPE_rot_rmse, rad2deg(metrics.RPE_rot_rmse));
    
    %% Generar gráficas
    
    % Figura 1: Trayectorias 2D
    figure('Name', sprintf('Trayectoria - %s', name), 'NumberTitle', 'off', ...
           'Position', [100, 100, 800, 600]);
    
    plot(gt_x, gt_y, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
    hold on;
    plot(est.x, est.y, 'b-', 'LineWidth', 1.2, 'DisplayName', name);
    
    % Marcar inicio y fin
    plot(gt_x(1), gt_y(1), 'go', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Inicio');
    plot(gt_x(end), gt_y(end), 'rx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Fin');
    
    axis equal; grid on;
    xlabel('x [m]', 'FontSize', 12);
    ylabel('y [m]', 'FontSize', 12);
    title(sprintf('Trayectoria 2D - %s vs Ground Truth', name), 'FontSize', 14);
    legend('Location', 'best');
    hold off;
    
    % Figura 2: Errores en el tiempo
    figure('Name', sprintf('Errores - %s', name), 'NumberTitle', 'off', ...
           'Position', [150, 150, 900, 700]);
    
    subplot(3, 1, 1);
    plot(est.t, err_pos, 'b-', 'LineWidth', 1);
    hold on;
    yline(metrics.ATE_mean, 'r--', sprintf('Media: %.3f m', metrics.ATE_mean), 'LineWidth', 1.5);
    hold off;
    grid on;
    ylabel('Error posición [m]', 'FontSize', 11);
    title(sprintf('Error de Posición (RMSE: %.4f m)', metrics.ATE_rmse), 'FontSize', 12);
    xlim([0, max(est.t)]);
    
    subplot(3, 1, 2);
    plot(est.t, rad2deg(err_yaw), 'b-', 'LineWidth', 1);
    hold on;
    yline(rad2deg(metrics.yaw_mean), 'r--', sprintf('Media: %.2f°', rad2deg(metrics.yaw_mean)), 'LineWidth', 1.5);
    yline(-rad2deg(metrics.yaw_mean), 'r--', 'LineWidth', 1.5);
    hold off;
    grid on;
    ylabel('Error yaw [°]', 'FontSize', 11);
    title(sprintf('Error de Orientación (RMSE: %.2f°)', rad2deg(metrics.yaw_rmse)), 'FontSize', 12);
    xlim([0, max(est.t)]);
    
    subplot(3, 1, 3);
    plot(est.t, err_x, 'r-', 'LineWidth', 1, 'DisplayName', 'Error X');
    hold on;
    plot(est.t, err_y, 'g-', 'LineWidth', 1, 'DisplayName', 'Error Y');
    hold off;
    grid on;
    xlabel('Tiempo [s]', 'FontSize', 11);
    ylabel('Error [m]', 'FontSize', 11);
    title('Errores por Componente', 'FontSize', 12);
    legend('Location', 'best');
    xlim([0, max(est.t)]);
    
    % Figura 3: Estados vs tiempo
    figure('Name', sprintf('Estados - %s', name), 'NumberTitle', 'off', ...
           'Position', [200, 100, 900, 700]);
    
    subplot(3, 1, 1);
    plot(gt.t, gt.x, 'k-', 'LineWidth', 1.5, 'DisplayName', 'GT');
    hold on;
    plot(est.t, est.x, 'b--', 'LineWidth', 1, 'DisplayName', name);
    hold off;
    grid on;
    ylabel('x [m]', 'FontSize', 11);
    title('Posición X', 'FontSize', 12);
    legend('Location', 'best');
    
    subplot(3, 1, 2);
    plot(gt.t, gt.y, 'k-', 'LineWidth', 1.5, 'DisplayName', 'GT');
    hold on;
    plot(est.t, est.y, 'b--', 'LineWidth', 1, 'DisplayName', name);
    hold off;
    grid on;
    ylabel('y [m]', 'FontSize', 11);
    title('Posición Y', 'FontSize', 12);
    legend('Location', 'best');
    
    subplot(3, 1, 3);
    plot(gt.t, rad2deg(gt.yaw), 'k-', 'LineWidth', 1.5, 'DisplayName', 'GT');
    hold on;
    plot(est.t, rad2deg(est.yaw), 'b--', 'LineWidth', 1, 'DisplayName', name);
    hold off;
    grid on;
    xlabel('Tiempo [s]', 'FontSize', 11);
    ylabel('yaw [°]', 'FontSize', 11);
    title('Orientación', 'FontSize', 12);
    legend('Location', 'best');
    
    fprintf('\nGráficas generadas.\n');
end
