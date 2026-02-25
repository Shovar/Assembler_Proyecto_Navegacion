function [map_binary, map_info] = create_alternative_map()
% CREATE_ALTERNATIVE_MAP - Crea un mapa alternativo para comparar
%
% Genera un mapa tipo laberinto con zonas abiertas y pasillos estrechos.
% Este mapa permite evaluar el comportamiento de A* y D* Lite en un
% entorno diferente al del SLAM, con mas restricciones de paso.
%
% Salidas:
%   map_binary : Matriz binaria (0=libre, 1=ocupado)
%   map_info   : Struct con metadatos
%
% Autores: Antonio Garcia Alcon, Adrian Santero Alonso, Johalex Jose Arrieta Perez

    resolution = 0.05;  % 5 cm por celda
    rows = 200;
    cols = 200;
    
    % Inicializar todo libre
    map_binary = zeros(rows, cols);
    
    % --- Paredes exteriores ---
    w = 2;
    map_binary(1:w, :) = 1;
    map_binary(end-w+1:end, :) = 1;
    map_binary(:, 1:w) = 1;
    map_binary(:, end-w+1:end) = 1;
    
    % --- Estructura tipo almacen industrial ---
    % Filas de estanterias (tipico de un almacen donde operaria un robot)
    
    % Estanterias horizontales (filas paralelas)
    shelf_rows = [35, 65, 95, 125, 155];
    shelf_width = 3;
    
    for i = 1:length(shelf_rows)
        r = shelf_rows(i);
        % Cada fila de estanterias con huecos para pasar
        map_binary(r:r+shelf_width, 15:80) = 1;
        map_binary(r:r+shelf_width, 95:160) = 1;
        
        % Solo algunas filas tienen estanterias en la zona derecha
        if mod(i, 2) == 0
            map_binary(r:r+shelf_width, 175:190) = 1;
        end
    end
    
    % --- Zona de carga/descarga (esquina inferior derecha) ---
    % Pared que delimita la zona
    map_binary(140:142, 130:198) = 1;
    % Puerta de acceso
    map_binary(140:142, 145:160) = 0;
    
    % Pallets/cajas en zona de carga
    map_binary(155:165, 140:150) = 1;
    map_binary(155:165, 160:170) = 1;
    map_binary(175:185, 145:155) = 1;
    map_binary(175:185, 165:180) = 1;
    
    % --- Oficina/control (esquina inferior izquierda) ---
    map_binary(160:162, 15:70) = 1;
    map_binary(160:162, 40:45) = 0;  % Puerta
    map_binary(162:198, 70:72) = 1;
    map_binary(178:183, 70:72) = 0;  % Puerta
    
    % Mesa dentro de la oficina
    map_binary(170:178, 25:45) = 1;
    
    % --- Columnas estructurales ---
    cols_pos = [50, 100, 150];
    rows_pos = [50, 100, 150];
    col_size = 3;
    
    for ci = 1:length(cols_pos)
        for ri = 1:length(rows_pos)
            r = rows_pos(ri);
            c = cols_pos(ci);
            if map_binary(r, c) == 0  % Solo si no hay ya obstaculo
                map_binary(r:r+col_size, c:c+col_size) = 1;
            end
        end
    end
    
    % Informacion del mapa
    map_info.resolution = resolution;
    map_info.origin = [0, 0];
    map_info.width_m = cols * resolution;
    map_info.height_m = rows * resolution;
    map_info.source = 'Alternativo (almacen industrial)';
    
    fprintf('    Mapa alternativo generado: entorno tipo almacen industrial\n');
end
