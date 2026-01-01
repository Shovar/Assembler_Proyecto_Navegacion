# Proyecto Integrador: Localizacion de Robot MiR100

Universidad Europea - Navegacion Autonoma de Robots
Diciembre 2025

## Descripcion del Proyecto

Este proyecto implementa un sistema de localizacion para robot movil usando un Filtro de Kalman Extendido (EKF) que fusiona datos de odometria e IMU.

La localizacion es el problema de determinar la posicion y orientacion (pose) de un robot en su entorno. Es fundamental para la navegacion autonoma porque el robot necesita saber donde esta para poder planificar rutas y evitar obstaculos.

Los sensores del robot no son perfectos: la odometria acumula error con el tiempo (drift), la IMU tiene ruido, y el laser puede tener lecturas erroneas. La solucion es fusionar multiples sensores para obtener una estimacion mas precisa.

## Objetivo

Implementar un algoritmo de localizacion que:
- Use datos reales de un robot MiR100
- Fusione odometria e IMU mediante EKF
- Evalue la precision comparando con ground truth
- Genere metricas y graficas para analisis

## Arquitectura del Sistema

El sistema funciona en dos etapas que se repiten continuamente:

SENSORES:
- Odometria (/odom): proporciona velocidad lineal (vx) y angular (w)
- IMU (/imu_data): proporciona orientacion (yaw) y velocidad angular (wz)

ALGORITMO EKF:

1. PREDICCION (usando odometria):
   - x' = x + v * cos(theta) * dt
   - y' = y + v * sin(theta) * dt
   - theta' = theta + w * dt

2. CORRECCION (usando IMU):
   - Calcula la Ganancia de Kalman: K = P * H' * inv(H * P * H' + R)
   - Corrige el estado: x = x' + K * (z - H * x')
   - Actualiza incertidumbre: P = (I - K * H) * P

SALIDA:
- Pose estimada [x, y, theta]

## Estructura de Archivos

- main_localization_v2.m: Script principal (ejecutar este)
- extract_all_data.m: Extrae datos del rosbag
- ekf_odom_imu.m: Algoritmo EKF (fusion odometria + IMU)
- evaluate_localization.m: Calcula metricas de error
- mir_basics_20251210_114529.bag: Rosbag con datos del robot
- analyze_bag.m: Analisis inicial del rosbag
- plot_scan.m: Visualizacion de scans laser
- README.md: Este archivo

## Como Ejecutar

Requisitos:
- MATLAB R2020b o superior
- ROS Toolbox (verificar con: ver('ros'))

Pasos:
1. Abrir MATLAB y navegar a esta carpeta
2. Ejecutar: main_localization_v2
3. Esperar los resultados (aproximadamente 1 minuto)

## Metricas de Evaluacion

ATE (Absolute Trajectory Error): Mide el error absoluto de posicion en cada instante.
Formula: ATE = sqrt(mean(||p_estimada - p_real||^2))
Valores buenos: menor a 0.5 m

RPE (Relative Pose Error): Mide el error en el movimiento relativo entre instantes.
Util para evaluar la consistencia del algoritmo.

Error de Yaw: Mide el error en la orientacion del robot.
Formula: Yaw Error = |theta_estimado - theta_real|
Valores buenos: menor a 2 grados

## Resultados Obtenidos

Metodo           | ATE RMSE [m] | Yaw RMSE [grados]
-----------------|--------------|------------------
Odometria pura   | 0.5745       | 1.89
EKF Odom+IMU     | 0.0669       | 0.62

Mejoras conseguidas:
- EKF mejoro el ATE en 88.3% respecto a odometria pura
- EKF mejoro el error de yaw en 67.3% respecto a odometria pura

## Teoria del EKF

El Filtro de Kalman Extendido es un algoritmo de estimacion que combina:
1. Modelo del sistema (como se mueve el robot)
2. Medidas de sensores (que observamos)

Para obtener la mejor estimacion posible del estado.

La Ganancia de Kalman pondera automaticamente:
- Si el sensor es muy preciso (R pequeno), K es grande y confiamos mas en la medida
- Si el modelo es muy preciso (P pequeno), K es pequeno y confiamos mas en la prediccion

Variables principales:
- mu: Estado estimado [x, y, theta]
- P: Matriz de covarianza (incertidumbre)
- G: Jacobiano del modelo de movimiento
- H: Jacobiano del modelo de medida
- Q: Ruido del proceso
- R: Ruido de medida
- K: Ganancia de Kalman

## Referencias Bibliograficas

Borenstein, J., Everett, H. R., & Feng, L. (1996). Where am I? Sensors and methods for mobile robot positioning. University of Michigan.

Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011). Introduction to autonomous mobile robots (2a ed.). MIT Press.

Thrun, S., Burgard, W., & Fox, D. (2005). Probabilistic robotics. MIT Press.

## Autores

Antonio Garcia Alcon
Adrian Santero Alonso
Johalex Jose Arrieta Perez

Universidad Europea
Navegacion Autonoma de Robots
Diciembre 2025
