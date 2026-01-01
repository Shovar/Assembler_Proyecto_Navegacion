function [est, debug] = ekf_odom_imu(data, params)
% EKF_ODOM_IMU - Filtro de Kalman Extendido fusionando Odometría e IMU
% Estado del robot: x = [x; y; theta]
%   - x, y: posición en metros
%   - theta: orientación (yaw) en radianes

% Modelo de predicción (odometría):
%   x(k+1) = x(k) + v * cos(theta) * dt
%   y(k+1) = y(k) + v * sin(theta) * dt
%   theta(k+1) = theta(k) + omega * dt

% Donde:
%   - v: velocidad lineal (de odometría)
%   - omega: velocidad angular (de odometría)
%   - dt: intervalo de tiempo

% Modelo de medida (IMU):
%   z = theta_imu (medida directa del yaw)


    fprintf('\n=== EKF ODOMETRÍA + IMU ===\n');
    fprintf('Fusionando datos de encoders e IMU...\n\n');


    % PASO 1: CONFIGURAR PARÁMETROS
    
    if nargin < 2
        params = struct();
    end
    
    % --- Ruido del modelo de movimiento (predicción) ---
    % Estos valores representan cuánto "confiamos" en la odometría
    % Valores más grandes = menos confianza = más incertidumbre
    if ~isfield(params, 'sigma_v')
        params.sigma_v = 0.1;      % Ruido en velocidad lineal [m/s]
    end
    if ~isfield(params, 'sigma_w')
        params.sigma_w = 0.05;     % Ruido en velocidad angular [rad/s]
    end
    
    % --- Ruido de la medida de IMU (corrección) ---
    % Valor más pequeño = confiamos más en la IMU
    if ~isfield(params, 'sigma_imu')
        params.sigma_imu = 0.02;   % Ruido del yaw de la IMU [rad]
    end
    
    fprintf('Parámetros del filtro:\n');
    fprintf('  σ_v (ruido vel. lineal):  %.3f m/s\n', params.sigma_v);
    fprintf('  σ_w (ruido vel. angular): %.3f rad/s\n', params.sigma_w);
    fprintf('  σ_imu (ruido IMU):        %.3f rad\n', params.sigma_imu);
    
   
    % PASO 2: INICIALIZACIÓN DEL FILTRO
    
    % Estado inicial: empezamos desde el ground truth
    % En un caso real, esto vendría de /initialpose o GPS
    mu = [data.gt.x(1);      % x inicial
          data.gt.y(1);      % y inicial  
          data.gt.yaw(1)];   % theta inicial
    
    % Covarianza inicial: poca incertidumbre porque conocemos el inicio
    % P es una matriz 3x3 (diagonal = varianza de cada estado)
    P = diag([0.01,    % Varianza en x [m²]
              0.01,    % Varianza en y [m²]
              0.01]);  % Varianza en theta [rad²]
    
    fprintf('\nEstado inicial:\n');
    fprintf('  Posición: (%.2f, %.2f) m\n', mu(1), mu(2));
    fprintf('  Orientación: %.2f°\n', rad2deg(mu(3)));
    
  
    % PASO 3: PREPARAR DATOS DE SENSORES
    
    % Número de muestras de odometría (nuestro "reloj" principal)
    nSteps = length(data.odom.t);
    
    % Pre-asignar memoria para resultados (más eficiente)
    est.t = zeros(nSteps, 1);
    est.x = zeros(nSteps, 1);
    est.y = zeros(nSteps, 1);
    est.yaw = zeros(nSteps, 1);
    
    % Debug: guardar evolución de incertidumbre
    debug.P_trace = zeros(nSteps, 1);      % Traza de P (incertidumbre total)
    debug.P_theta = zeros(nSteps, 1);      % Incertidumbre en theta
    debug.innovations = zeros(nSteps, 1);  % Diferencia predicción vs medida
    debug.num_corrections = 0;
    
    % Interpolar datos de IMU a los tiempos de odometría
    % (porque la IMU puede tener diferente frecuencia)
    imu_yaw_interp = interp1(data.imu.t, data.imu.yaw, data.odom.t, 'linear', 'extrap');
    
    fprintf('\nDatos preparados:\n');
    fprintf('  Pasos de odometría: %d\n', nSteps);
    fprintf('  Datos IMU interpolados: OK\n');
    
    % PASO 4: BUCLE PRINCIPAL DEL EKF
    
    fprintf('\nEjecutando EKF...\n');
    
    for k = 1:nSteps
        
        
        % ETAPA A: PREDICCIÓN
     
        % Usamos el modelo de movimiento para predecir el nuevo estado
        
        if k > 1
            % Calcular dt (tiempo desde el paso anterior)
            dt = data.odom.t(k) - data.odom.t(k-1);
            
            % Obtener velocidades de la odometría
            v = data.odom.vx(k);      % Velocidad lineal [m/s]
            w = data.odom.vyaw(k);    % Velocidad angular [rad/s]
            
            % --- Predecir nuevo estado ---
            theta = mu(3);  % Orientación actual
            
            % Ecuaciones cinemáticas del robot diferencial
            mu_pred = [mu(1) + v * cos(theta) * dt;   % x nuevo
                       mu(2) + v * sin(theta) * dt;   % y nuevo
                       mu(3) + w * dt];               % theta nuevo
            
            % --- Calcular Jacobiano del modelo de movimiento ---
            % El Jacobiano G representa cómo pequeños cambios en el estado
            % actual afectan al estado predicho
            % G = ∂f/∂x donde f es el modelo de movimiento
            G = [1, 0, -v * sin(theta) * dt;
                 0, 1,  v * cos(theta) * dt;
                 0, 0,  1];
            
            % --- Matriz de ruido del proceso ---
            % Q representa la incertidumbre añadida por el movimiento
            % Cuanto más rápido va el robot, más ruido hay
            Q = diag([(params.sigma_v * dt)^2,    % Ruido en x
                      (params.sigma_v * dt)^2,    % Ruido en y
                      (params.sigma_w * dt)^2]);  % Ruido en theta
            
            % --- Actualizar covarianza (propagación de incertidumbre) ---
            % La incertidumbre crece con el movimiento
            P_pred = G * P * G' + Q;
            
            % Guardar predicción
            mu = mu_pred;
            P = P_pred;
        end
        
       
        % ETAPA B: CORRECCIÓN CON IMU
        
        % Usamos la medida de la IMU para corregir nuestra estimación
        
        % Medida de la IMU: yaw directamente
        z_imu = imu_yaw_interp(k);
        
        % --- Modelo de medida ---
        % H es el Jacobiano de la función de medida
        % Como medimos theta directamente: h(x) = theta, entonces H = [0, 0, 1]
        H = [0, 0, 1];
        
        % --- Ruido de medida ---
        R = params.sigma_imu^2;  % Varianza del sensor IMU
        
        % --- Innovación (residuo) ---
        % Es la diferencia entre lo que medimos y lo que predecimos
        y_innov = wrapToPi(z_imu - mu(3));  % wrapToPi maneja el wrap-around de ángulos
        
        % --- Covarianza de la innovación ---
        S = H * P * H' + R;
        
        % --- Ganancia de Kalman ---
        % K determina cuánto "creemos" a la medida vs la predicción
        % Si R es pequeño (buena IMU), K será grande y corregimos más
        % Si P es pequeño (buena predicción), K será pequeño y corregimos menos
        K = P * H' / S;
        
        % --- Actualizar estado ---
        mu = mu + K * y_innov;
        
        % --- Actualizar covarianza ---
        % Usamos la forma de Joseph para estabilidad numérica
        I_KH = eye(3) - K * H;
        P = I_KH * P * I_KH' + K * R * K';
        
        debug.num_corrections = debug.num_corrections + 1;
        
        
        % ETAPA C: NORMALIZAR Y GUARDAR
        
        % Normalizar ángulo a [-pi, pi]
        mu(3) = wrapToPi(mu(3));
        
        % Guardar resultados
        est.t(k) = data.odom.t(k);
        est.x(k) = mu(1);
        est.y(k) = mu(2);
        est.yaw(k) = mu(3);
        
        % Guardar debug
        debug.P_trace(k) = trace(P);
        debug.P_theta(k) = P(3, 3);
        debug.innovations(k) = y_innov;
        
        % Mostrar progreso cada 2000 pasos
        if mod(k, 2000) == 0
            fprintf('  Progreso: %d/%d (%.1f%%)\n', k, nSteps, 100*k/nSteps);
        end
    end
    
   
    % PASO 5: RESUMEN FINAL
    
    fprintf('\n=== EKF COMPLETADO ===\n');
    fprintf('Correcciones aplicadas: %d\n', debug.num_corrections);
    fprintf('Incertidumbre final (traza P): %.4f\n', trace(P));
    fprintf('Estado final: (%.2f, %.2f) m, %.2f°\n', ...
        mu(1), mu(2), rad2deg(mu(3)));
    
end
