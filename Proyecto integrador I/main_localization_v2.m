%% MAIN_LOCALIZATION_V2.m
% Proyecto Integrador: Localizacion de Robot Movil
% Universidad Europea - Navegacion Autonoma de Robots

% Robot: MiR100
% Metodo: EKF (Extended Kalman Filter) fusionando Odometria e IMU

% Este script implementa un sistema de localizacion que fusiona dos
% fuentes de informacion:
%   1. ODOMETRIA: Buena estimacion de posicion (x, y) pero acumula error en yaw
%   2. IMU: Medidas directas de orientacion, corrige el drift del yaw

% El algoritmo EKF combina ambas fuentes de manera optima, ponderando
% cada una segun su nivel de incertidumbre.

clear; clc; close all;

disp('------------------------------------------------------------');
disp('   PROYECTO INTEGRADOR - LOCALIZACION DE ROBOT MOVIL');
disp('   Robot MiR100 | EKF: Fusion Odometria + IMU');
disp('------------------------------------------------------------');
disp(' ');

%% CONFIGURACION

bagFile = 'mir_basics_20251210_114529.bag';

% Parametros del EKF (ajustables para mejorar resultados)
params.sigma_v = 0.1;      % Ruido velocidad lineal [m/s]
params.sigma_w = 0.05;     % Ruido velocidad angular [rad/s]
params.sigma_imu = 0.02;   % Ruido de la IMU [rad]

%% PASO 1: EXTRACCION DE DATOS DEL ROSBAG
% El rosbag contiene todos los datos grabados del robot durante su
% operacion. Extraemos odometria, IMU, ground truth, scans y mapa.

disp('PASO 1: Extraccion de datos del rosbag');
disp('------------------------------------------------------------');

data = extract_all_data(bagFile);

% Mostrar resumen
fprintf('\nResumen de datos extraidos:\n');
fprintf('  Duracion del experimento: %.1f segundos (%.1f minutos)\n', ...
    max(data.odom.t), max(data.odom.t)/60);
fprintf('  Frecuencia de odometria: %.1f Hz\n', ...
    length(data.odom.t) / max(data.odom.t));
fprintf('  Frecuencia de IMU: %.1f Hz\n', ...
    length(data.imu.t) / max(data.imu.t));

%% PASO 2: VISUALIZACION DE DATOS DE ENTRADA
% Antes de procesar, visualizamos los datos para entender el problema

disp(' ');
disp('PASO 2: Visualizacion de datos de entrada');
disp('------------------------------------------------------------');

% Figura 1: Mapa del entorno con trayectoria
figure('Name', 'Mapa del Entorno', 'NumberTitle', 'off', ...
       'Position', [50, 400, 700, 500]);

xLim = [double(data.map.originX), ...
        double(data.map.originX) + double(data.map.width) * data.map.resolution];
yLim = [double(data.map.originY), ...
        double(data.map.originY) + double(data.map.height) * data.map.resolution];
imagesc(xLim, yLim, data.map.data);
colormap(flipud(gray));
axis xy equal;
hold on;
plot(data.gt.x, data.gt.y, 'g-', 'LineWidth', 2, 'DisplayName', 'Trayectoria GT');
plot(data.gt.x(1), data.gt.y(1), 'go', 'MarkerSize', 15, 'LineWidth', 3, ...
    'DisplayName', 'Inicio');
plot(data.gt.x(end), data.gt.y(end), 'rx', 'MarkerSize', 15, 'LineWidth', 3, ...
    'DisplayName', 'Fin');
hold off;
xlabel('x [m]');
ylabel('y [m]');
title('Mapa de Ocupacion con Trayectoria Real');
legend('Location', 'best');
colorbar;

disp('  Figura 1: Mapa del entorno generado');

% Figura 2: Comparacion Odometria vs Ground Truth
figure('Name', 'Odometria vs Ground Truth', 'NumberTitle', 'off', ...
       'Position', [100, 350, 800, 550]);

plot(data.gt.x, data.gt.y, 'k-', 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
hold on;
plot(data.odom.x, data.odom.y, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Odometria');
plot(data.gt.x(1), data.gt.y(1), 'go', 'MarkerSize', 12, 'LineWidth', 3, ...
    'DisplayName', 'Inicio');
plot(data.gt.x(end), data.gt.y(end), 'rx', 'MarkerSize', 12, 'LineWidth', 3, ...
    'DisplayName', 'Fin');
hold off;
axis equal; grid on;
xlabel('x [m]');
ylabel('y [m]');
title('Comparacion: Odometria vs Ground Truth');
legend('Location', 'best');

disp('  Figura 2: Comparacion odometria vs GT generada');

% Figura 3: Datos de la IMU
figure('Name', 'Datos de IMU', 'NumberTitle', 'off', ...
       'Position', [150, 300, 900, 400]);

subplot(1, 2, 1);
plot(data.imu.t, rad2deg(data.imu.yaw), 'b-', 'LineWidth', 1);
hold on;
plot(data.gt.t, rad2deg(data.gt.yaw), 'k--', 'LineWidth', 1.5);
hold off;
grid on;
xlabel('Tiempo [s]');
ylabel('Yaw [grados]');
title('Orientacion: IMU vs Ground Truth');
legend('IMU', 'Ground Truth', 'Location', 'best');

subplot(1, 2, 2);
plot(data.imu.t, data.imu.wz, 'r-', 'LineWidth', 1);
grid on;
xlabel('Tiempo [s]');
ylabel('wz [rad/s]');
title('Velocidad Angular (IMU)');

disp('  Figura 3: Datos de IMU generados');

%% PASO 3: EJECUTAR ALGORITMO EKF
% Aqui aplicamos el filtro de Kalman extendido para fusionar odometria e IMU

disp(' ');
disp('PASO 3: Ejecutar localizacion EKF');
disp('------------------------------------------------------------');

tic;
[est_ekf, debug_ekf] = ekf_odom_imu(data, params);
tiempo_ekf = toc;

fprintf('\nTiempo de ejecucion: %.2f segundos\n', tiempo_ekf);

%% PASO 4: EVALUAR RESULTADOS
% Comparamos nuestra estimacion con el ground truth usando metricas
% estandar de la literatura (ATE, RPE)

disp(' ');
disp('PASO 4: Evaluacion de resultados');
disp('------------------------------------------------------------');

% Evaluar EKF Odom+IMU
fprintf('\nEvaluando EKF (Odometria + IMU)...\n');
metrics_ekf = evaluate_localization(est_ekf, data.gt, 'EKF Odom+IMU');

% Evaluar Odometria pura (baseline)
fprintf('\nEvaluando Odometria pura (baseline)...\n');
odom_est.t = data.odom.t;
odom_est.x = data.odom.x;
odom_est.y = data.odom.y;
odom_est.yaw = data.odom.yaw;
metrics_odom = evaluate_localization(odom_est, data.gt, 'Odometria pura');

% Evaluar AMCL (si existe, para comparacion)
metrics_amcl = [];
if ~isempty(data.amcl) && length(data.amcl.t) > 10
    fprintf('\nEvaluando AMCL (referencia del robot)...\n');
    
    % Eliminar timestamps duplicados del AMCL
    [unique_t, unique_idx] = unique(data.amcl.t);
    
    if length(unique_t) > 10
        amcl_interp.t = data.odom.t;
        amcl_interp.x = interp1(unique_t, data.amcl.x(unique_idx), data.odom.t, 'linear', 'extrap');
        amcl_interp.y = interp1(unique_t, data.amcl.y(unique_idx), data.odom.t, 'linear', 'extrap');
        amcl_interp.yaw = interp1(unique_t, unwrap(data.amcl.yaw(unique_idx)), data.odom.t, 'linear', 'extrap');
        amcl_interp.yaw = wrapToPi(amcl_interp.yaw);
        
        metrics_amcl = evaluate_localization(amcl_interp, data.gt, 'AMCL');
    end
end

%% PASO 5: TABLA COMPARATIVA DE RESULTADOS

disp(' ');
disp('PASO 5: Tabla comparativa de resultados');
disp('------------------------------------------------------------');
fprintf('\n');
fprintf('Metodo             | ATE RMSE [m] | Yaw RMSE [deg] | RPE Trans [m]\n');
fprintf('-------------------|--------------|----------------|---------------\n');
fprintf('Odometria pura     |    %6.4f    |     %6.2f     |    %6.4f\n', ...
    metrics_odom.ATE_rmse, rad2deg(metrics_odom.yaw_rmse), metrics_odom.RPE_trans_rmse);
fprintf('EKF Odom+IMU       |    %6.4f    |     %6.2f     |    %6.4f\n', ...
    metrics_ekf.ATE_rmse, rad2deg(metrics_ekf.yaw_rmse), metrics_ekf.RPE_trans_rmse);
if ~isempty(metrics_amcl)
 fprintf('AMCL (referencia)  |    %6.4f    |     %6.2f     |    %6.4f\n', ...
    metrics_amcl.ATE_rmse, rad2deg(metrics_amcl.yaw_rmse), metrics_amcl.RPE_trans_rmse);
end
fprintf('\n');

% Calcular mejora
mejora_ate = (metrics_odom.ATE_rmse - metrics_ekf.ATE_rmse) / metrics_odom.ATE_rmse * 100;
mejora_yaw = (metrics_odom.yaw_rmse - metrics_ekf.yaw_rmse) / metrics_odom.yaw_rmse * 100;

fprintf('Analisis de mejora:\n');
if mejora_ate > 0
    fprintf('  - EKF mejoro el ATE en %.1f%% respecto a odometria pura\n', mejora_ate);
else
    fprintf('  - EKF empeoro el ATE en %.1f%% respecto a odometria pura\n', -mejora_ate);
end
if mejora_yaw > 0
    fprintf('  - EKF mejoro el error de yaw en %.1f%% respecto a odometria pura\n', mejora_yaw);
else
    fprintf('  - EKF empeoro el error de yaw en %.1f%% respecto a odometria pura\n', -mejora_yaw);
end

%% PASO 6: GRAFICAS FINALES COMPARATIVAS

disp(' ');
disp('PASO 6: Generando graficas finales');
disp('------------------------------------------------------------');

% Figura Principal: Comparacion de Trayectorias
figure('Name', 'Comparacion Final de Trayectorias', 'NumberTitle', 'off', ...
       'Position', [200, 100, 1000, 700]);

subplot(2, 2, [1, 3]);
plot(data.gt.x, data.gt.y, 'k-', 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
hold on;
plot(data.odom.x, data.odom.y, 'b--', 'LineWidth', 1.2, 'DisplayName', 'Odometria');
plot(est_ekf.x, est_ekf.y, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF Odom+IMU');
plot(data.gt.x(1), data.gt.y(1), 'go', 'MarkerSize', 12, 'LineWidth', 3, ...
    'DisplayName', 'Inicio');
plot(data.gt.x(end), data.gt.y(end), 'kx', 'MarkerSize', 12, 'LineWidth', 3, ...
    'DisplayName', 'Fin');
hold off;
axis equal; grid on;
xlabel('x [m]');
ylabel('y [m]');
title('Comparacion de Trayectorias');
legend('Location', 'best');

% Grafico de barras: ATE
subplot(2, 2, 2);
bar_data = [metrics_odom.ATE_rmse, metrics_ekf.ATE_rmse];
bar_labels = {'Odometria', 'EKF'};
b = bar(bar_data, 'FaceColor', 'flat');
b.CData(1,:) = [0.3 0.5 0.8];
b.CData(2,:) = [0.8 0.3 0.3];
set(gca, 'XTickLabel', bar_labels);
ylabel('ATE RMSE [m]');
title('Error de Trayectoria Absoluto');
grid on;

text(1, bar_data(1)+0.02, sprintf('%.3f m', bar_data(1)), ...
    'HorizontalAlignment', 'center', 'FontSize', 10);
text(2, bar_data(2)+0.02, sprintf('%.3f m', bar_data(2)), ...
    'HorizontalAlignment', 'center', 'FontSize', 10);

% Grafico de barras: Yaw Error
subplot(2, 2, 4);
bar_data = rad2deg([metrics_odom.yaw_rmse, metrics_ekf.yaw_rmse]);
b = bar(bar_data, 'FaceColor', 'flat');
b.CData(1,:) = [0.3 0.5 0.8];
b.CData(2,:) = [0.8 0.3 0.3];
set(gca, 'XTickLabel', bar_labels);
ylabel('Yaw RMSE [grados]');
title('Error de Orientacion');
grid on;

text(1, bar_data(1)+0.1, sprintf('%.2f deg', bar_data(1)), ...
    'HorizontalAlignment', 'center', 'FontSize', 10);
text(2, bar_data(2)+0.1, sprintf('%.2f deg', bar_data(2)), ...
    'HorizontalAlignment', 'center', 'FontSize', 10);

disp('  Figura de comparacion final generada');

% Figura: Evolucion de la Incertidumbre
figure('Name', 'Evolucion de Incertidumbre EKF', 'NumberTitle', 'off', ...
       'Position', [250, 150, 800, 400]);

subplot(1, 2, 1);
plot(est_ekf.t, debug_ekf.P_trace, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]');
ylabel('Traza(P)');
title('Incertidumbre Total del EKF');

subplot(1, 2, 2);
plot(est_ekf.t, rad2deg(sqrt(debug_ekf.P_theta)), 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]');
ylabel('sigma theta [grados]');
title('Incertidumbre en Orientacion');

disp('  Figura de incertidumbre generada');

%% PASO 7: GUARDAR RESULTADOS

disp(' ');
disp('PASO 7: Guardando resultados');
disp('------------------------------------------------------------');

results.est_ekf = est_ekf;
results.debug_ekf = debug_ekf;
results.metrics_ekf = metrics_ekf;
results.metrics_odom = metrics_odom;
results.params = params;
results.data = data;

save('localization_results_v2.mat', 'results');
disp('  Resultados guardados en: localization_results_v2.mat');

%% RESUMEN FINAL

disp(' ');
disp('------------------------------------------------------------');
disp('   LOCALIZACION COMPLETADA');
disp('------------------------------------------------------------');
fprintf('  Mejor ATE RMSE:  %.4f m (EKF Odom+IMU)\n', metrics_ekf.ATE_rmse);
fprintf('  Mejor Yaw RMSE:  %.2f grados (EKF Odom+IMU)\n', rad2deg(metrics_ekf.yaw_rmse));
disp('------------------------------------------------------------');
