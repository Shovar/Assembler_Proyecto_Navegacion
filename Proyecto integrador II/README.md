# Proyecto Integrador: SLAM - Robot MiR100

Universidad Europea - Navegacion Autonoma de Robots
Diciembre 2025

## Descripcion del Proyecto

Este proyecto implementa un sistema de construccion de mapas (SLAM - Simultaneous Localization and Mapping) para el robot MiR100 usando datos reales de sensores.

El sistema construye un mapa de ocupacion (Occupancy Grid) utilizando las lecturas del sensor laser y las poses del robot estimadas mediante el Filtro de Kalman Extendido (EKF) desarrollado en el Proyecto de Localizacion.

## Conexion con el Proyecto de Localizacion

Este proyecto esta directamente conectado con el proyecto anterior:

- Proyecto 1 (Localizacion): Estima donde esta el robot usando odometria + IMU
- Proyecto 2 (SLAM): Usa esas poses + laser para construir el mapa del entorno

El flujo completo es:
1. Odometria + IMU -> EKF -> Poses estimadas [x, y, theta]
2. Poses + Scans laser -> Occupancy Grid Mapping -> Mapa

## Objetivo

Implementar un sistema de mapeo que:
- Use datos reales del robot MiR100
- Reutilice las poses del EKF del proyecto de localizacion
- Construya un mapa de ocupacion del entorno
- Compare el mapa generado con el mapa de referencia del rosbag

## Arquitectura del Sistema

SENSORES:
- Laser (/scan): 10,003 lecturas con 541 rayos cada una
- Odometria (/odom): velocidades para prediccion
- IMU (/imu_data): orientacion para correccion

ALGORITMO:

1. LOCALIZACION (EKF del Proyecto 1):
   - Predice pose con odometria
   - Corrige con IMU
   - Genera poses [x, y, theta] para cada instante

2. MAPEO (Occupancy Grid):
   - Para cada scan laser:
     a. Obtiene la pose del robot en ese instante
     b. Transforma los rayos laser a coordenadas globales
     c. Actualiza las celdas del mapa usando modelo de sensor inverso
   - Celdas libres: el rayo paso sin impactar
   - Celdas ocupadas: el rayo impacto en un obstaculo

SALIDA:
- Mapa de ocupacion (Occupancy Grid)

## Estructura de Archivos

- main_slam.m: Script principal (ejecutar este)
- extract_slam_data.m: Extrae scans, poses y mapa de referencia del rosbag
- ekf_localization.m: EKF para estimar poses (del proyecto de localizacion)
- build_occupancy_map.m: Construye el mapa de ocupacion
- evaluate_map.m: Calcula metricas de calidad del mapa
- mir_basics_20251210_114529.bag: Rosbag con datos del robot (No se encuentra en el repositorio por tamaño excesivo, recuperar de los materiales de la entrega)
- README.md: Este archivo

## Como Ejecutar

Requisitos:
- MATLAB R2020b o superior
- ROS Toolbox

Pasos:
1. Colocar todos los archivos .m y el .bag en la misma carpeta
2. Abrir MATLAB y navegar a esa carpeta
3. Ejecutar: main_slam
4. Esperar los resultados (aproximadamente 2-3 minutos)

## Tipo de Mapa

Tipo: Occupancy Grid (Rejilla de Ocupacion)
Resolucion: 0.05 m/celda (5 cm)
Tamano: 400x400 celdas
Rango: X [-5, 15] m, Y [-5, 15] m
Marco de referencia: Mundo (world frame)

Representacion:
- Probabilidad 0.0 = Libre (blanco)
- Probabilidad 0.5 = Desconocido (gris)
- Probabilidad 1.0 = Ocupado (negro)

## Metricas de Evaluacion

Accuracy: Porcentaje de celdas clasificadas correctamente.

Precision: De las celdas que marcamos como ocupadas, cuantas lo eran realmente.

Recall: De las celdas que eran ocupadas, cuantas detectamos.

IoU (Intersection over Union): Medida de solapamiento entre el mapa generado y el de referencia.

## Teoria del Occupancy Grid Mapping

El mapeo con rejilla de ocupacion es un algoritmo clasico descrito en "Probabilistic Robotics" (Thrun et al., 2005, Capitulo 9).

Cada celda del mapa almacena la probabilidad de estar ocupada. Se actualiza usando el modelo de sensor inverso:

Para cada rayo laser:
1. Las celdas por donde pasa el rayo se marcan como LIBRES
2. La celda donde impacta el rayo se marca como OCUPADA

La actualizacion se hace en log-odds para evitar problemas numericos:
- l(m|z) = l(m) + l_sensor - l_prior

Donde:
- l(m) = log(p/(1-p)) es el log-odds actual
- l_sensor es la evidencia del sensor
- l_prior es la probabilidad a priori

## Referencias Bibliograficas

Borenstein, J., Everett, H. R., & Feng, L. (1996). Where am I? Sensors and methods for mobile robot positioning. University of Michigan.

Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011). Introduction to autonomous mobile robots (2a ed.). MIT Press.

Thrun, S., Burgard, W., & Fox, D. (2005). Probabilistic robotics. MIT Press.
- Capitulo 9: Occupancy Grid Mapping

## Autores

Antonio Garcia Alcon
Adrian Santero Alonso
Johalex Jose Arrieta Perez

Universidad Europea
Navegacion Autonoma de Robots
Diciembre 2025
