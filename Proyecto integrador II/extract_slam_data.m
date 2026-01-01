function [scans, odom, imu, gt, map_ref] = extract_slam_data(bagFile)
% EXTRACT_SLAM_DATA - Extrae todos los datos necesarios para SLAM
%
% Entradas:
%   bagFile - Ruta al archivo .bag
%
% Salidas:
%   scans   - Estructura con datos del laser
%   odom    - Estructura con datos de odometria
%   imu     - Estructura con datos de IMU
%   gt      - Estructura con ground truth
%   map_ref - Estructura con mapa de referencia

    % Cargar rosbag
    bag = rosbag(bagFile);
    
    % Funciones auxiliares
    timeFromHeader = @(h) double(h.Stamp.Sec) + double(h.Stamp.Nsec)*1e-9;
    quatToYaw = @(q) atan2(2*(q.W*q.Z + q.X*q.Y), 1 - 2*(q.Y^2 + q.Z^2));
    
    %% Extraer scans del laser (/scan)
    fprintf('  Extrayendo scans laser...\n');
    bagScan = select(bag, 'Topic', '/scan');
    scanMsgs = readMessages(bagScan, 'DataFormat', 'struct');
    
    n_scans = numel(scanMsgs);
    
    % Obtener info del primer scan
    s1 = scanMsgs{1};
    n_rays = numel(s1.Ranges);
    angles = double(s1.AngleMin + (0:n_rays-1)' * s1.AngleIncrement);
    
    scans.ranges = cell(n_scans, 1);
    scans.timestamps = zeros(n_scans, 1);
    scans.angles = angles;
    scans.angle_min = s1.AngleMin;
    scans.angle_max = s1.AngleMax;
    scans.range_min = s1.RangeMin;
    scans.range_max = s1.RangeMax;
    
    for k = 1:n_scans
        msg = scanMsgs{k};
        scans.ranges{k} = double(msg.Ranges(:));
        scans.timestamps(k) = timeFromHeader(msg.Header);
    end
    
    scans.n = n_scans;
    scans.duration = scans.timestamps(end) - scans.timestamps(1);
    
    %% Extraer odometria (/odom)
    fprintf('  Extrayendo odometria...\n');
    bagOdom = select(bag, 'Topic', '/odom');
    odomMsgs = readMessages(bagOdom, 'DataFormat', 'struct');
    
    n_odom = numel(odomMsgs);
    odom.t = zeros(n_odom, 1);
    odom.x = zeros(n_odom, 1);
    odom.y = zeros(n_odom, 1);
    odom.yaw = zeros(n_odom, 1);
    odom.vx = zeros(n_odom, 1);
    odom.wz = zeros(n_odom, 1);
    
    for k = 1:n_odom
        msg = odomMsgs{k};
        odom.t(k) = timeFromHeader(msg.Header);
        odom.x(k) = msg.Pose.Pose.Position.X;
        odom.y(k) = msg.Pose.Pose.Position.Y;
        odom.yaw(k) = quatToYaw(msg.Pose.Pose.Orientation);
        odom.vx(k) = msg.Twist.Twist.Linear.X;
        odom.wz(k) = msg.Twist.Twist.Angular.Z;
    end
    odom.n = n_odom;
    
    %% Extraer IMU (/imu_data)
    fprintf('  Extrayendo IMU...\n');
    bagIMU = select(bag, 'Topic', '/imu_data');
    imuMsgs = readMessages(bagIMU, 'DataFormat', 'struct');
    
    n_imu = numel(imuMsgs);
    imu.t = zeros(n_imu, 1);
    imu.yaw = zeros(n_imu, 1);
    imu.wz = zeros(n_imu, 1);
    
    for k = 1:n_imu
        msg = imuMsgs{k};
        imu.t(k) = timeFromHeader(msg.Header);
        imu.yaw(k) = quatToYaw(msg.Orientation);
        imu.wz(k) = msg.AngularVelocity.Z;
    end
    imu.n = n_imu;
    
    %% Extraer Ground Truth (/base_pose_ground_truth)
    fprintf('  Extrayendo ground truth...\n');
    bagGT = select(bag, 'Topic', '/base_pose_ground_truth');
    gtMsgs = readMessages(bagGT, 'DataFormat', 'struct');
    
    n_gt = numel(gtMsgs);
    gt.t = zeros(n_gt, 1);
    gt.x = zeros(n_gt, 1);
    gt.y = zeros(n_gt, 1);
    gt.yaw = zeros(n_gt, 1);
    
    for k = 1:n_gt
        msg = gtMsgs{k};
        gt.t(k) = timeFromHeader(msg.Header);
        gt.x(k) = msg.Pose.Pose.Position.X;
        gt.y(k) = msg.Pose.Pose.Position.Y;
        gt.yaw(k) = quatToYaw(msg.Pose.Pose.Orientation);
    end
    gt.n = n_gt;
    
    %% Extraer mapa de referencia (/map)
    fprintf('  Extrayendo mapa de referencia...\n');
    bagMap = select(bag, 'Topic', '/map');
    mapMsg = readMessages(bagMap, 1, 'DataFormat', 'struct');
    mapMsg = mapMsg{1};
    
    map_ref.width = double(mapMsg.Info.Width);
    map_ref.height = double(mapMsg.Info.Height);
    map_ref.resolution = double(mapMsg.Info.Resolution);
    map_ref.origin_x = double(mapMsg.Info.Origin.Position.X);
    map_ref.origin_y = double(mapMsg.Info.Origin.Position.Y);
    
    % Convertir datos del mapa (valores: -1=desconocido, 0=libre, 100=ocupado)
    map_data = double(reshape(mapMsg.Data, map_ref.width, map_ref.height)');
    
    % Crear rangos de coordenadas
    map_ref.x_range = map_ref.origin_x + (0:map_ref.width-1) * map_ref.resolution;
    map_ref.y_range = map_ref.origin_y + (0:map_ref.height-1) * map_ref.resolution;
    
    map_ref.data = map_data;
    
    fprintf('  Extraccion completada.\n');
end
