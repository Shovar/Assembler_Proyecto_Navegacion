function map_updated = add_dynamic_obstacle(map, center, obs_size)
% ADD_DYNAMIC_OBSTACLE - Anade un obstaculo dinamico al mapa
%
% Simula la aparicion de un obstaculo que no estaba en el mapa original
% (por ejemplo, una persona, un objeto movil, una puerta cerrada, etc.)
%
% Entradas:
%   map      : Mapa binario original
%   center   : [fila, col] centro del obstaculo
%   obs_size : Tamanio del obstaculo (se crea un cuadrado de obs_size x obs_size)
%
% Salida:
%   map_updated : Mapa con el obstaculo anadido
%
% Autores: Antonio Garcia Alcon, Adrian Santero Alonso, Johalex Jose Arrieta Perez

    map_updated = map;
    [rows, cols] = size(map);
    
    % Calcular los limites del obstaculo
    half = floor(obs_size / 2);
    r_min = max(1, center(1) - half);
    r_max = min(rows, center(1) + half);
    c_min = max(1, center(2) - half);
    c_max = min(cols, center(2) + half);
    
    % Colocar el obstaculo
    map_updated(r_min:r_max, c_min:c_max) = 1;
end
