function metrics = evaluate_map(map_ekf, map_gt, map_ref)
% EVALUATE_MAP - Evalua la calidad de los mapas generados
%
% Compara los mapas generados con el mapa de referencia del rosbag
% usando metricas estandar de clasificacion.
%
% Entradas:
%   map_ekf - Mapa generado con poses EKF
%   map_gt  - Mapa generado con poses Ground Truth
%   map_ref - Mapa de referencia del rosbag
%
% Salidas:
%   metrics - Estructura con las metricas calculadas

    % Umbral para binarizar mapas
    thresh_generated = 0.6;  % Probabilidad > 0.6 = ocupado
    thresh_ref = 50;         % Valor > 50 en mapa referencia = ocupado
    
    % Binarizar mapas generados
    map_ekf_bin = map_ekf.prob > thresh_generated;
    map_gt_bin = map_gt.prob > thresh_generated;
    
    % Redimensionar mapa de referencia para que coincida
    % Primero necesitamos alinear las coordenadas
    
    % Obtener region comun
    x_min_common = max(map_ekf.x_range(1), map_ref.x_range(1));
    x_max_common = min(map_ekf.x_range(end), map_ref.x_range(end));
    y_min_common = max(map_ekf.y_range(1), map_ref.y_range(1));
    y_max_common = min(map_ekf.y_range(end), map_ref.y_range(end));
    
    % Crear grilla comun
    res = map_ekf.resolution;
    x_common = x_min_common:res:x_max_common;
    y_common = y_min_common:res:y_max_common;
    
    % Interpolar mapas a la grilla comun
    [X_ekf, Y_ekf] = meshgrid(map_ekf.x_range, map_ekf.y_range);
    [X_gt, Y_gt] = meshgrid(map_gt.x_range, map_gt.y_range);
    [X_ref, Y_ref] = meshgrid(map_ref.x_range, map_ref.y_range);
    [X_common, Y_common] = meshgrid(x_common, y_common);
    
    % Interpolar
    map_ekf_interp = interp2(X_ekf, Y_ekf, double(map_ekf_bin), X_common, Y_common, 'nearest', 0);
    map_gt_interp = interp2(X_gt, Y_gt, double(map_gt_bin), X_common, Y_common, 'nearest', 0);
    map_ref_interp = interp2(X_ref, Y_ref, double(map_ref.data > thresh_ref), X_common, Y_common, 'nearest', 0);
    
    % Convertir a logico
    map_ekf_interp = map_ekf_interp > 0.5;
    map_gt_interp = map_gt_interp > 0.5;
    map_ref_interp = map_ref_interp > 0.5;
    
    % Calcular metricas para EKF vs Referencia
    metrics.ekf_vs_ref = calculate_metrics(map_ekf_interp, map_ref_interp);
    
    % Calcular metricas para GT vs Referencia
    metrics.gt_vs_ref = calculate_metrics(map_gt_interp, map_ref_interp);
    
    % Calcular metricas para EKF vs GT
    metrics.ekf_vs_gt = calculate_metrics(map_ekf_interp, map_gt_interp);
    
    % Informacion adicional
    metrics.common_region.x_range = x_common;
    metrics.common_region.y_range = y_common;
    metrics.common_region.size = [length(y_common), length(x_common)];
end


function m = calculate_metrics(pred, truth)
% CALCULATE_METRICS - Calcula metricas de clasificacion
%
% Entradas:
%   pred  - Mapa predicho (binario)
%   truth - Mapa verdadero (binario)
%
% Salidas:
%   m - Estructura con metricas

    % True Positives: ambos dicen ocupado
    TP = sum(pred(:) & truth(:));
    
    % True Negatives: ambos dicen libre
    TN = sum(~pred(:) & ~truth(:));
    
    % False Positives: pred dice ocupado, truth dice libre
    FP = sum(pred(:) & ~truth(:));
    
    % False Negatives: pred dice libre, truth dice ocupado
    FN = sum(~pred(:) & truth(:));
    
    % Total de celdas
    total = TP + TN + FP + FN;
    
    % Accuracy: porcentaje de celdas correctas
    m.accuracy = (TP + TN) / total;
    
    % Precision: de las que dijimos ocupadas, cuantas lo eran realmente
    if (TP + FP) > 0
        m.precision = TP / (TP + FP);
    else
        m.precision = 0;
    end
    
    % Recall: de las que eran ocupadas, cuantas detectamos
    if (TP + FN) > 0
        m.recall = TP / (TP + FN);
    else
        m.recall = 0;
    end
    
    % F1 Score: media armonica de precision y recall
    if (m.precision + m.recall) > 0
        m.f1 = 2 * m.precision * m.recall / (m.precision + m.recall);
    else
        m.f1 = 0;
    end
    
    % IoU (Intersection over Union): para celdas ocupadas
    if (TP + FP + FN) > 0
        m.iou = TP / (TP + FP + FN);
    else
        m.iou = 0;
    end
    
    % Guardar valores brutos tambien
    m.TP = TP;
    m.TN = TN;
    m.FP = FP;
    m.FN = FN;
end
