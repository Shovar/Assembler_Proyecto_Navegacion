%% MAIN_SLAM.M - Sistema de Mapeo para Robot MiR100
% Universidad Europea - Navegacion Autonoma de Robots
% Proyecto Integrador - SLAM
%
% Este script construye un mapa de ocupacion (Occupancy Grid) usando
% los datos del laser y las poses estimadas por el EKF del proyecto
% de localizacion.
%
% Conexion con el Proyecto de Localizacion:
%   - Proyecto 1: Calculamos donde esta el robot (poses con EKF)
%   - Proyecto 2: Usamos esas poses + laser para construir el mapa
%
% Autores: Antonio Garcia Alcon, Adrian Santero Alonso, Johalex Jose Arrieta Perez
% Fecha: Diciembre 2025

clc; clear; close all;

fprintf('=============================================================\n');
fprintf('   PROYECTO INTEGRADOR - SLAM\n');
fprintf('   Robot MiR100 - Occupancy Grid Mapping\n');
fprintf('=============================================================\n\n');

%% PASO 1: Cargar datos del rosbag
fprintf('PASO 1: Extrayendo datos del rosbag...\n');
fprintf('-------------------------------------------------------------\n');

bagFile = 'mir_basics_20251210_114529.bag';

if ~isfile(bagFile)
    error('No se encuentra el archivo %s', bagFile);
end

tic;
[scans, odom, imu, gt, map_ref] = extract_slam_data(bagFile);
t_extract = toc;

fprintf('Datos extraidos en %.1f segundos:\n', t_extract);
fprintf('  - Scans laser: %d lecturas (%.1f Hz)\n', scans.n, scans.n / scans.duration);
fprintf('  - Odometria: %d mensajes\n', odom.n);
fprintf('  - IMU: %d mensajes\n', imu.n);
fprintf('  - Ground Truth: %d poses\n', gt.n);
fprintf('  - Mapa referencia: %dx%d pixeles (%.2f m/pixel)\n', ...
    map_ref.width, map_ref.height, map_ref.resolution);
fprintf('\n');

%% PASO 2: Ejecutar EKF para obtener poses (del proyecto de localizacion)
fprintf('PASO 2: Calculando poses con EKF (Proyecto de Localizacion)...\n');
fprintf('-------------------------------------------------------------\n');

% Parametros del EKF (los mismos del proyecto de localizacion)
params.sigma_v = 0.1;
params.sigma_w = 0.05;
params.sigma_imu = 0.02;

% Ejecutar EKF
tic;
ekf_poses = ekf_localization(odom, imu, params);
t_ekf = toc;

fprintf('EKF ejecutado en %.2f segundos\n', t_ekf);
fprintf('  - Poses estimadas: %d\n', length(ekf_poses.t));
fprintf('\n');

%% PASO 3: Construir el mapa de ocupacion
fprintf('PASO 3: Construyendo mapa de ocupacion...\n');
fprintf('-------------------------------------------------------------\n');

% Parametros del mapa
map_params.resolution = 0.05;    % 5 cm por celda (igual que el mapa de referencia)
map_params.x_min = -5;
map_params.x_max = 15;
map_params.y_min = -5;
map_params.y_max = 15;
map_params.p_occ = 0.7;          % Probabilidad de ocupado si el rayo impacta
map_params.p_free = 0.3;         % Probabilidad de libre si el rayo pasa
map_params.p_prior = 0.5;        % Probabilidad inicial (desconocido)

% Construir mapa usando poses del EKF
tic;
map_ekf = build_occupancy_map(scans, ekf_poses, map_params);
t_map_ekf = toc;
fprintf('Mapa con poses EKF construido en %.1f segundos\n', t_map_ekf);

% Construir mapa usando Ground Truth (para comparacion)
tic;
map_gt = build_occupancy_map(scans, gt, map_params);
t_map_gt = toc;
fprintf('Mapa con Ground Truth construido en %.1f segundos\n', t_map_gt);

fprintf('\n');

%% PASO 4: Evaluar calidad del mapa
fprintf('PASO 4: Evaluando calidad del mapa...\n');
fprintf('-------------------------------------------------------------\n');

% Comparar mapas
metrics = evaluate_map(map_ekf, map_gt, map_ref);

fprintf('\nMetricas de calidad:\n');
fprintf('  Mapa EKF vs Mapa Referencia:\n');
fprintf('    - Accuracy: %.2f %%\n', metrics.ekf_vs_ref.accuracy * 100);
fprintf('    - Precision: %.2f %%\n', metrics.ekf_vs_ref.precision * 100);
fprintf('    - Recall: %.2f %%\n', metrics.ekf_vs_ref.recall * 100);
fprintf('    - IoU (Intersection over Union): %.2f %%\n', metrics.ekf_vs_ref.iou * 100);

fprintf('\n  Mapa GT vs Mapa Referencia:\n');
fprintf('    - Accuracy: %.2f %%\n', metrics.gt_vs_ref.accuracy * 100);
fprintf('    - Precision: %.2f %%\n', metrics.gt_vs_ref.precision * 100);
fprintf('    - Recall: %.2f %%\n', metrics.gt_vs_ref.recall * 100);
fprintf('    - IoU (Intersection over Union): %.2f %%\n', metrics.gt_vs_ref.iou * 100);

fprintf('\n');

%% PASO 5: Visualizar resultados
fprintf('PASO 5: Generando visualizaciones...\n');
fprintf('-------------------------------------------------------------\n');

% Figura 1: Mapa de referencia del rosbag
figure('Name', 'Mapa de Referencia', 'NumberTitle', 'off');
imagesc(map_ref.x_range, map_ref.y_range, map_ref.data);
colormap(flipud(gray));
axis equal tight;
set(gca, 'YDir', 'normal');
xlabel('X [m]');
ylabel('Y [m]');
title('Mapa de Referencia (del rosbag)');
colorbar;

% Figura 2: Mapa construido con Ground Truth
figure('Name', 'Mapa con Ground Truth', 'NumberTitle', 'off');
imagesc(map_gt.x_range, map_gt.y_range, map_gt.prob);
colormap(flipud(gray));
axis equal tight;
set(gca, 'YDir', 'normal');
xlabel('X [m]');
ylabel('Y [m]');
title('Mapa Construido con Poses Ground Truth');
colorbar;
hold on;
plot(gt.x(1:50:end), gt.y(1:50:end), 'r.', 'MarkerSize', 3);
legend('Trayectoria GT', 'Location', 'best');

% Figura 3: Mapa construido con EKF
figure('Name', 'Mapa con EKF', 'NumberTitle', 'off');
imagesc(map_ekf.x_range, map_ekf.y_range, map_ekf.prob);
colormap(flipud(gray));
axis equal tight;
set(gca, 'YDir', 'normal');
xlabel('X [m]');
ylabel('Y [m]');
title('Mapa Construido con Poses EKF (Nuestro Metodo)');
colorbar;
hold on;
plot(ekf_poses.x(1:50:end), ekf_poses.y(1:50:end), 'b.', 'MarkerSize', 3);
legend('Trayectoria EKF', 'Location', 'best');

% Figura 4: Comparacion lado a lado
figure('Name', 'Comparacion de Mapas', 'NumberTitle', 'off');

subplot(1,3,1);
imagesc(map_ref.x_range, map_ref.y_range, map_ref.data);
colormap(flipud(gray));
axis equal tight;
set(gca, 'YDir', 'normal');
xlabel('X [m]'); ylabel('Y [m]');
title('Referencia');

subplot(1,3,2);
imagesc(map_gt.x_range, map_gt.y_range, map_gt.prob);
colormap(flipud(gray));
axis equal tight;
set(gca, 'YDir', 'normal');
xlabel('X [m]'); ylabel('Y [m]');
title('Ground Truth');

subplot(1,3,3);
imagesc(map_ekf.x_range, map_ekf.y_range, map_ekf.prob);
colormap(flipud(gray));
axis equal tight;
set(gca, 'YDir', 'normal');
xlabel('X [m]'); ylabel('Y [m]');
title('EKF (Nuestro)');

sgtitle('Comparacion de Mapas de Ocupacion');

% Figura 5: Mapa EKF con trayectoria
figure('Name', 'Mapa EKF con Trayectoria', 'NumberTitle', 'off');

imagesc(map_ekf.x_range, map_ekf.y_range, map_ekf.prob);
colormap(flipud(gray));
axis equal tight;
set(gca, 'YDir', 'normal');
xlabel('X [m]'); ylabel('Y [m]');
title('Mapa Construido con EKF y Trayectoria del Robot');
colorbar;
hold on;
plot(ekf_poses.x, ekf_poses.y, 'b-', 'LineWidth', 1.5);
plot(ekf_poses.x(1), ekf_poses.y(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(ekf_poses.x(end), ekf_poses.y(end), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
legend('Trayectoria', 'Inicio', 'Fin', 'Location', 'best');

% Figura 6: Evolucion del mapa (4 momentos)
figure('Name', 'Evolucion del Mapa', 'NumberTitle', 'off');

n_scans = length(scans.ranges);
indices = round([0.25, 0.5, 0.75, 1.0] * n_scans);

for i = 1:4
    subplot(2,2,i);
    
    % Construir mapa parcial
    scans_partial = scans;
    scans_partial.ranges = scans.ranges(1:indices(i));
    scans_partial.angles = scans.angles;
    scans_partial.timestamps = scans.timestamps(1:indices(i));
    scans_partial.n = indices(i);
    
    map_partial = build_occupancy_map(scans_partial, gt, map_params);
    
    imagesc(map_partial.x_range, map_partial.y_range, map_partial.prob);
    colormap(flipud(gray));
    axis equal tight;
    set(gca, 'YDir', 'normal');
    xlabel('X [m]'); ylabel('Y [m]');
    title(sprintf('Scan %d de %d (%.0f%%)', indices(i), n_scans, 100*i/4));
end
sgtitle('Evolucion del Mapa durante el Recorrido');

fprintf('Figuras generadas.\n\n');

%% PASO 6: Guardar resultados
fprintf('PASO 6: Guardando resultados...\n');
fprintf('-------------------------------------------------------------\n');

results.scans = scans;
results.odom = odom;
results.imu = imu;
results.gt = gt;
results.ekf_poses = ekf_poses;
results.map_ref = map_ref;
results.map_ekf = map_ekf;
results.map_gt = map_gt;
results.metrics = metrics;
results.params = map_params;

save('slam_results.mat', 'results');
fprintf('Resultados guardados en slam_results.mat\n\n');

%% RESUMEN FINAL
fprintf('=============================================================\n');
fprintf('   RESUMEN DE RESULTADOS\n');
fprintf('=============================================================\n');
fprintf('Mapa generado:\n');
fprintf('  - Tipo: Occupancy Grid (Rejilla de Ocupacion)\n');
fprintf('  - Resolucion: %.2f m/celda\n', map_params.resolution);
fprintf('  - Tamano: %dx%d celdas\n', size(map_ekf.prob, 2), size(map_ekf.prob, 1));
fprintf('  - Rango X: [%.1f, %.1f] m\n', map_params.x_min, map_params.x_max);
fprintf('  - Rango Y: [%.1f, %.1f] m\n', map_params.y_min, map_params.y_max);
fprintf('\nCalidad del mapa (EKF vs Referencia):\n');
fprintf('  - Accuracy: %.2f %%\n', metrics.ekf_vs_ref.accuracy * 100);
fprintf('  - IoU: %.2f %%\n', metrics.ekf_vs_ref.iou * 100);
fprintf('\nTiempos de ejecucion:\n');
fprintf('  - Extraccion de datos: %.1f s\n', t_extract);
fprintf('  - EKF localizacion: %.2f s\n', t_ekf);
fprintf('  - Construccion mapa EKF: %.1f s\n', t_map_ekf);
fprintf('  - Construccion mapa GT: %.1f s\n', t_map_gt);
fprintf('=============================================================\n');
