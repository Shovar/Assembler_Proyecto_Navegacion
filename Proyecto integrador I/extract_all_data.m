function data = extract_all_data(bagFile)
% EXTRACT_ALL_DATA Extrae todos los datos relevantes del rosbag MiR100

% Uso:
%   data = extract_all_data('mir_basics_20251210_114529.bag')

% Salida:
%   data - Estructura con campos: odom, gt, amcl, scans, map, imu

    fprintf('=== EXTRACCIÓN DE DATOS DEL ROSBAG ===\n\n');
    
    %% Cargar bag
    if nargin < 1
        [f, p] = uigetfile('*.bag', 'Selecciona un rosbag');
        if isequal(f,0), error('No se seleccionó archivo.'); end
        bagFile = fullfile(p, f);
    end
    
    fprintf('Cargando: %s\n', bagFile);
    bag = rosbag(bagFile);
    
    % Helpers
    hasTopic = @(name) any(strcmp(name, bag.AvailableTopics.Properties.RowNames));
    timeFromHeader = @(h) double(h.Stamp.Sec) + double(h.Stamp.Nsec)*1e-9;
    quatToYaw = @(q) atan2(2*(q.W*q.Z + q.X*q.Y), 1 - 2*(q.Y^2 + q.Z^2));
    
    %% 1. ODOMETRÍA (/odom)
    fprintf('\n[1/6] Extrayendo odometría...\n');
    data.odom = [];
    if hasTopic('/odom')
        bagSel = select(bag, 'Topic', '/odom');
        msgs = readMessages(bagSel, 'DataFormat', 'struct');
        n = numel(msgs);
        
        data.odom.t = zeros(n,1);
        data.odom.x = zeros(n,1);
        data.odom.y = zeros(n,1);
        data.odom.yaw = zeros(n,1);
        data.odom.vx = zeros(n,1);
        data.odom.vyaw = zeros(n,1);
        
        for k = 1:n
            m = msgs{k};
            data.odom.t(k) = timeFromHeader(m.Header);
            data.odom.x(k) = m.Pose.Pose.Position.X;
            data.odom.y(k) = m.Pose.Pose.Position.Y;
            data.odom.yaw(k) = quatToYaw(m.Pose.Pose.Orientation);
            data.odom.vx(k) = m.Twist.Twist.Linear.X;
            data.odom.vyaw(k) = m.Twist.Twist.Angular.Z;
        end
        fprintf('   ✓ %d mensajes de /odom\n', n);
    end
    
    %% 2. GROUND TRUTH (/base_pose_ground_truth)
    fprintf('[2/6] Extrayendo ground truth...\n');
    data.gt = [];
    if hasTopic('/base_pose_ground_truth')
        bagSel = select(bag, 'Topic', '/base_pose_ground_truth');
        msgs = readMessages(bagSel, 'DataFormat', 'struct');
        n = numel(msgs);
        
        data.gt.t = zeros(n,1);
        data.gt.x = zeros(n,1);
        data.gt.y = zeros(n,1);
        data.gt.yaw = zeros(n,1);
        
        for k = 1:n
            m = msgs{k};
            data.gt.t(k) = timeFromHeader(m.Header);
            data.gt.x(k) = m.Pose.Pose.Position.X;
            data.gt.y(k) = m.Pose.Pose.Position.Y;
            data.gt.yaw(k) = quatToYaw(m.Pose.Pose.Orientation);
        end
        fprintf('   ✓ %d mensajes de /base_pose_ground_truth\n', n);
    end
    
    %% 3. AMCL (/amcl_pose) - Para comparación
    fprintf('[3/6] Extrayendo AMCL (referencia)...\n');
    data.amcl = [];
    if hasTopic('/amcl_pose')
        bagSel = select(bag, 'Topic', '/amcl_pose');
        msgs = readMessages(bagSel, 'DataFormat', 'struct');
        n = numel(msgs);
        
        data.amcl.t = zeros(n,1);
        data.amcl.x = zeros(n,1);
        data.amcl.y = zeros(n,1);
        data.amcl.yaw = zeros(n,1);
        
        for k = 1:n
            m = msgs{k};
            data.amcl.t(k) = timeFromHeader(m.Header);
            data.amcl.x(k) = m.Pose.Pose.Position.X;
            data.amcl.y(k) = m.Pose.Pose.Position.Y;
            data.amcl.yaw(k) = quatToYaw(m.Pose.Pose.Orientation);
        end
        fprintf('   ✓ %d mensajes de /amcl_pose\n', n);
    end
    
    %% 4. SCANS LÁSER (/scan)
    fprintf('[4/6] Extrayendo scans láser...\n');
    data.scans = [];
    if hasTopic('/scan')
        bagSel = select(bag, 'Topic', '/scan');
        msgs = readMessages(bagSel, 'DataFormat', 'struct');
        n = numel(msgs);
        
        % Obtener info del primer scan
        s1 = msgs{1};
        numRays = numel(s1.Ranges);
        angles = double(s1.AngleMin + (0:numRays-1)' * s1.AngleIncrement);
        
        data.scans.t = zeros(n,1);
        data.scans.ranges = zeros(numRays, n);
        data.scans.angles = angles;
        data.scans.rangeMin = s1.RangeMin;
        data.scans.rangeMax = s1.RangeMax;
        
        for k = 1:n
            m = msgs{k};
            data.scans.t(k) = timeFromHeader(m.Header);
            data.scans.ranges(:,k) = double(m.Ranges);
        end
        fprintf('   ✓ %d scans, %d rayos cada uno\n', n, numRays);
    end
    
    %% 5. MAPA (/map)
    fprintf('[5/6] Extrayendo mapa...\n');
    data.map = [];
    if hasTopic('/map')
        bagSel = select(bag, 'Topic', '/map');
        msgs = readMessages(bagSel, 'DataFormat', 'struct');
        m = msgs{end}; % Último mapa
        
        data.map.resolution = m.Info.Resolution;
        data.map.width = m.Info.Width;
        data.map.height = m.Info.Height;
        data.map.originX = m.Info.Origin.Position.X;
        data.map.originY = m.Info.Origin.Position.Y;
        data.map.data = reshape(double(m.Data), m.Info.Width, m.Info.Height)';
        
        fprintf('   ✓ Mapa %dx%d, resolución %.3f m/pixel\n', ...
            data.map.width, data.map.height, data.map.resolution);
    end
    
    %% 6. IMU (/imu_data)
    fprintf('[6/6] Extrayendo IMU...\n');
    data.imu = [];
    if hasTopic('/imu_data')
        bagSel = select(bag, 'Topic', '/imu_data');
        msgs = readMessages(bagSel, 'DataFormat', 'struct');
        n = numel(msgs);
        
        data.imu.t = zeros(n,1);
        data.imu.ax = zeros(n,1);
        data.imu.ay = zeros(n,1);
        data.imu.wz = zeros(n,1);  % velocidad angular en z
        data.imu.yaw = zeros(n,1);
        
        for k = 1:n
            m = msgs{k};
            data.imu.t(k) = timeFromHeader(m.Header);
            data.imu.ax(k) = m.LinearAcceleration.X;
            data.imu.ay(k) = m.LinearAcceleration.Y;
            data.imu.wz(k) = m.AngularVelocity.Z;
            data.imu.yaw(k) = quatToYaw(m.Orientation);
        end
        fprintf('   ✓ %d mensajes de IMU\n', n);
    end
    
    %% Normalizar tiempos (empezar en t=0)
    t0 = min([data.odom.t(1), data.gt.t(1), data.scans.t(1)]);
    data.odom.t = data.odom.t - t0;
    data.gt.t = data.gt.t - t0;
    data.scans.t = data.scans.t - t0;
    if ~isempty(data.amcl), data.amcl.t = data.amcl.t - t0; end
    if ~isempty(data.imu), data.imu.t = data.imu.t - t0; end
    
    %% Resumen
    fprintf('\n=== EXTRACCIÓN COMPLETADA ===\n');
    fprintf('Duración total: %.1f segundos\n', max(data.gt.t));
    fprintf('Pose inicial GT: [%.2f, %.2f, %.2f°]\n', ...
        data.gt.x(1), data.gt.y(1), rad2deg(data.gt.yaw(1)));
    
end
