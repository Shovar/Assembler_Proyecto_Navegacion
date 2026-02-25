function free_cell = find_free_cell(map, pos)
% FIND_FREE_CELL - Busca la celda libre mas cercana a una posicion dada
%
% Util cuando un punto de inicio o meta cae sobre un obstaculo y
% necesitamos ajustarlo a la celda libre mas proxima.
%
% Entradas:
%   map : Mapa binario (0=libre, 1=ocupado)
%   pos : [fila, col] posicion original
%
% Salida:
%   free_cell : [fila, col] celda libre mas cercana

    [rows, cols] = size(map);
    
    for radius = 1:max(rows, cols)
        for dr = -radius:radius
            for dc = -radius:radius
                % Solo buscar en el perimetro del cuadrado (optimizacion)
                if abs(dr) ~= radius && abs(dc) ~= radius
                    continue;
                end
                
                nr = pos(1) + dr;
                nc = pos(2) + dc;
                
                if nr >= 1 && nr <= rows && nc >= 1 && nc <= cols
                    if map(nr, nc) == 0
                        free_cell = [nr, nc];
                        return;
                    end
                end
            end
        end
    end
    
    warning('No se encontro celda libre cerca de [%d, %d]', pos(1), pos(2));
    free_cell = pos;  % Devolver la posicion original como fallback
end
