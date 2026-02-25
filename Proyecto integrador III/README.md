# Proyecto Integrador III - Planificación de Trayectorias
# Navegación Autónoma de Robots - Universidad Europea

### Autores
- Antonio García Alcón
- Adrián Santero Alonso
- Johalex José Arrieta Pérez

### Fecha
Febrero 2026

---

## Descripción

Este es el **Proyecto Integrador III** de la asignatura Navegación Autónoma de Robots. Implementa un sistema de planificación y replanificación de trayectorias para un robot móvil sobre mapas 2D de ocupación, utilizando los algoritmos **A*** para planificación inicial y **D* Lite** para replanificación ante obstáculos dinámicos.

Este proyecto es la continuación directa de los proyectos anteriores:
- **Proyecto Integrador I (Localización):** EKF Odom+IMU → Estimación de pose del robot MiR100
- **Proyecto Integrador II (SLAM):** Occupancy Grid Mapping → Mapa 2D del entorno
- **Proyecto Integrador III (este):** A* + D* Lite → Ruta navegable con replanificación dinámica

## Estructura de archivos

```
proyecto integrador III/
├── main_planning.m              # Script principal (ejecutar este)
├── load_slam_map.m              # Carga el mapa del SLAM (Proyecto II)
├── create_alternative_map.m     # Genera mapa alternativo para comparar
├── astar.m                      # Algoritmo A* (planificación inicial)
├── dstar_lite.m                 # Algoritmo D* Lite (replanificación)
├── add_dynamic_obstacle.m       # Simula obstáculos dinámicos
├── evaluate_path.m              # Métricas de calidad de trayectorias
├── plot_map_with_path.m         # Visualiza mapa con ruta planificada
├── plot_replanning.m            # Visualiza escenario de replanificación
├── plot_metrics_comparison.m    # Gráfica comparativa de métricas
├── plot_results.m               # Funciones de visualización (auxiliar)
├── find_free_cell.m             # Función auxiliar
├── README.md                    # Este archivo
├── Informe_Proyecto_Integrador_III.docx  # Informe del proyecto
└── slam_map.mat                 # (Opcional) Mapa guardado del Proyecto II
```

## Requisitos

- MATLAB R2020a o superior
- Image Processing Toolbox (para `bwdist` en evaluación de clearance)
- Robotics System Toolbox (opcional, solo si se carga el rosbag directamente)

## Ejecución

1. Abrir MATLAB y navegar a la carpeta del proyecto
2. Si tienes el mapa del SLAM, guardarlo como `slam_map.mat` con la variable
   `occupancy_grid` (matriz binaria 0/1)
3. Ejecutar:

```matlab
>> main_planning
```

El script ejecutará automáticamente:
1. Carga de ambos mapas (SLAM y alternativo)
2. Planificación A* en ambos mapas
3. Simulación de obstáculos dinámicos (parcial y total)
4. Replanificación D* Lite para cada escenario
5. Evaluación de métricas
6. Generación y guardado de 7 figuras PNG
7. Tabla resumen de resultados en consola

Las figuras se guardan automáticamente como PNG en la carpeta del proyecto.

## Algoritmos implementados

### A* (Hart, Nilsson & Raphael, 1968)
- Conectividad 8 (cardinales + diagonales)
- Heurística: distancia euclidiana (admisible y consistente)
- Verificación de corte de esquinas en diagonales

### D* Lite (Koenig & Likhachev, 2002)
- Búsqueda hacia atrás (desde meta hacia inicio)
- Actualización incremental ante cambios en el mapa
- Replanificación eficiente reutilizando información previa

## Métricas evaluadas

| Métrica | Descripción |
|---------|-------------|
| Longitud | Distancia total de la ruta (metros) |
| Suavidad | Suma de cambios angulares (menor = mejor) |
| Clearance | Distancia media a obstáculos |
| Tiempo | Tiempo de cómputo del algoritmo |
| Nodos | Número de nodos expandidos |
