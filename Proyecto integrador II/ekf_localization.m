function poses = ekf_localization(odom, imu, params)
% EKF_LOCALIZATION - Filtro de Kalman Extendido para localizacion
%
% Esta funcion es del Proyecto de Localizacion. La reutilizamos aqui
% para obtener las poses del robot que usaremos en el mapeo.
%
% Entradas:
%   odom   - Datos de odometria (t, vx, wz)
%   imu    - Datos de IMU (t, yaw)
%   params - Parametros del filtro (sigma_v, sigma_w, sigma_imu)
%
% Salidas:
%   poses  - Estructura con poses estimadas (t, x, y, yaw)

    % Interpolamos IMU a los tiempos de odometria
    imu_yaw_interp = interp1(imu.t, unwrap(imu.yaw), odom.t, 'linear', 'extrap');
    
    % Estado inicial desde la primera pose de odometria
    mu = [odom.x(1); odom.y(1); odom.yaw(1)];
    
    % Covarianza inicial
    P = diag([0.1, 0.1, 0.05]);
    
    % Numero de pasos
    N = length(odom.t);
    
    % Almacenar resultados
    poses.t = odom.t;
    poses.x = zeros(N, 1);
    poses.y = zeros(N, 1);
    poses.yaw = zeros(N, 1);
    
    poses.x(1) = mu(1);
    poses.y(1) = mu(2);
    poses.yaw(1) = mu(3);
    
    % Modelo de medida: observamos directamente theta
    H = [0, 0, 1];
    
    % Ruido de medida (IMU)
    R = params.sigma_imu^2;
    
    % Bucle principal del EKF
    for k = 2:N
        % Tiempo transcurrido
        dt = odom.t(k) - odom.t(k-1);
        
        if dt <= 0 || dt > 1
            poses.x(k) = poses.x(k-1);
            poses.y(k) = poses.y(k-1);
            poses.yaw(k) = poses.yaw(k-1);
            continue;
        end
        
        % Entrada de control (velocidades de odometria)
        v = odom.vx(k);
        w = odom.wz(k);
        
        theta = mu(3);
        
        % === PREDICCION ===
        % Modelo cinematico diferencial
        mu_pred = [
            mu(1) + v * cos(theta) * dt;
            mu(2) + v * sin(theta) * dt;
            mu(3) + w * dt
        ];
        
        % Jacobiano del modelo de movimiento
        G = [1, 0, -v * sin(theta) * dt;
             0, 1,  v * cos(theta) * dt;
             0, 0,  1];
        
        % Ruido del proceso
        Q = diag([params.sigma_v^2 * dt^2, ...
                  params.sigma_v^2 * dt^2, ...
                  params.sigma_w^2 * dt^2]);
        
        % Propagar covarianza
        P_pred = G * P * G' + Q;
        
        % === CORRECCION ===
        % Medida de la IMU (yaw)
        z = imu_yaw_interp(k);
        
        % Innovacion
        y_innov = z - mu_pred(3);
        y_innov = atan2(sin(y_innov), cos(y_innov));  % Normalizar angulo
        
        % Ganancia de Kalman
        S = H * P_pred * H' + R;
        K = P_pred * H' / S;
        
        % Actualizar estado
        mu = mu_pred + K * y_innov;
        mu(3) = atan2(sin(mu(3)), cos(mu(3)));  % Normalizar angulo
        
        % Actualizar covarianza
        P = (eye(3) - K * H) * P_pred;
        
        % Guardar resultado
        poses.x(k) = mu(1);
        poses.y(k) = mu(2);
        poses.yaw(k) = mu(3);
    end
    
    poses.n = N;
end
