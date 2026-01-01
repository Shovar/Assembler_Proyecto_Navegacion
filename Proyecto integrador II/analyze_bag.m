function analyze_bag(bagFile)
% Script básico para analizar un rosbag de MIR
%
% Uso:
%   analyze_mir_bag('ruta/al/rosbag.bag')

    %----------------------------
    % 0) Cargar bag
    %----------------------------
    if nargin < 1
        [f, p] = uigetfile('*.bag', 'Selecciona un rosbag');
        if isequal(f,0)
            disp('No se ha seleccionado archivo.');
            return;
        end
        bagFile = fullfile(p, f);
    end

    fprintf('Cargando rosbag: %s\n', bagFile);
    bag = rosbag(bagFile);

    fprintf('\nTopics disponibles:\n');
    disp(bag.AvailableTopics.Properties.RowNames);

    % Helper para comprobar si existe un topic
    hasTopic = @(name) any(strcmp(name, bag.AvailableTopics.Properties.RowNames));

    % Helper para tiempo (segundos)
    timeFromHeader = @(h) double(h.Stamp.Sec) + double(h.Stamp.Nsec)*1e-9;

    % Helper para yaw desde cuaternión
    quatToYaw = @(q) atan2( ...
        2*(q.W*q.Z + q.X*q.Y), ...
        1 - 2*(q.Y^2 + q.Z^2));

    %----------------------------
    % 1) ODOMETRÍA (/odom)
    %----------------------------
    odom = [];
    if hasTopic('/odom')
        bagOdom = select(bag, 'Topic', '/odom');
        odomMsgs = readMessages(bagOdom, 'DataFormat', 'struct');

        n = numel(odomMsgs);
        odom.t   = zeros(n,1);
        odom.x   = zeros(n,1);
        odom.y   = zeros(n,1);
        odom.yaw = zeros(n,1);

        for k = 1:n
            m = odomMsgs{k};
            odom.t(k) = timeFromHeader(m.Header);
            odom.x(k) = m.Pose.Pose.Position.X;
            odom.y(k) = m.Pose.Pose.Position.Y;

            q = m.Pose.Pose.Orientation;
            odom.yaw(k) = quatToYaw(q);
        end
        fprintf('Leídos %d mensajes de /odom\n', n);
    else
        warning('No se ha encontrado /odom en el bag.');
    end

    %----------------------------
    % 2) GROUND TRUTH (/base_pose_ground_truth)
    %----------------------------
    gt = [];
    if hasTopic('/base_pose_ground_truth')
        bagGT = select(bag, 'Topic', '/base_pose_ground_truth');
        gtMsgs = readMessages(bagGT, 'DataFormat', 'struct');
    
        n = numel(gtMsgs);
        gt.t   = zeros(n,1);
        gt.x   = zeros(n,1);
        gt.y   = zeros(n,1);
        gt.yaw = zeros(n,1);
    
        for k = 1:n
            m = gtMsgs{k};
            gt.t(k) = timeFromHeader(m.Header);
    
            % OJO: aquí es Pose, no pose
            gt.x(k) = m.Pose.Pose.Position.X;
            gt.y(k) = m.Pose.Pose.Position.Y;
    
            q = m.Pose.Pose.Orientation;
            gt.yaw(k) = quatToYaw(q);
        end
        fprintf('Leídos %d mensajes de /base_pose_ground_truth\n', n);
    else
        fprintf('No hay /base_pose_ground_truth (no se pintará GT).\n');
    end


    %----------------------------
    % 3) AMCL (/amcl_pose)
    %----------------------------
    amcl = [];
    if hasTopic('/amcl_pose')
        bagAmcl = select(bag, 'Topic', '/amcl_pose');
        amclMsgs = readMessages(bagAmcl, 'DataFormat', 'struct');

        n = numel(amclMsgs);
        amcl.t   = zeros(n,1);
        amcl.x   = zeros(n,1);
        amcl.y   = zeros(n,1);
        amcl.yaw = zeros(n,1);

        for k = 1:n
            m = amclMsgs{k};
            amcl.t(k) = timeFromHeader(m.Header);
            amcl.x(k) = m.Pose.Pose.Position.X;
            amcl.y(k) = m.Pose.Pose.Position.Y;

            q = m.Pose.Pose.Orientation;
            amcl.yaw(k) = quatToYaw(q);
        end
        fprintf('Leídos %d mensajes de /amcl_pose\n', n);
    else
        fprintf('No hay /amcl_pose (no se pintará AMCL).\n');
    end

    %----------------------------
    % 4) PLOTS BÁSICOS
    %----------------------------

    % 4.1 Trayectoria XY
    figure('Name','Trayectorias','NumberTitle','off');
    hold on; grid on; axis equal;
    if ~isempty(gt)
        plot(gt.x, gt.y, 'k-', 'DisplayName', 'Ground truth');
    end
    if ~isempty(odom)
        plot(odom.x, odom.y, 'b--', 'DisplayName', 'Odom');
    end
    if ~isempty(amcl)
        plot(amcl.x, amcl.y, 'r-', 'DisplayName', 'AMCL');
    end
    xlabel('x [m]');
    ylabel('y [m]');
    title('Trayectorias 2D');
    legend('Location','best');
    hold off;

    % 4.2 Yaw vs tiempo (si hay odom y/o amcl)
    if ~isempty(odom) || ~isempty(amcl) || ~isempty(gt)
        figure('Name','Yaw vs tiempo','NumberTitle','off');
        hold on; grid on;
        if ~isempty(odom)
            plot(odom.t - odom.t(1), odom.yaw, 'b--', 'DisplayName', 'Odom');
        end
        if ~isempty(amcl)
            plot(amcl.t - amcl.t(1), amcl.yaw, 'r-', 'DisplayName', 'AMCL');
        end
        if ~isempty(gt)
            plot(gt.t - gt.t(1), gt.yaw, 'k-', 'DisplayName', 'GT');
        end
        xlabel('t [s]');
        ylabel('yaw [rad]');
        title('Orientación (yaw) vs tiempo');
        legend('Location','best');
        hold off;
    end


    fprintf('\nListo. Revisa las figuras generadas.\n');

end
