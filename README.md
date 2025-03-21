# Memoria de Estimación de estados en MATLAB

Este repositorio contiene archivos relacionados con la memoria “estimación de estados de vehículos terrestres usando estimación de horizonte móvil con  mediciones redundantes de GPS de bajo coste incorporando restricciones geométricas”.

## Contenido del Repositorio

### Archivos MATLAB (`.m`)
- **main_memoria.m**: Archivo principal encargado de estimar, simular y calcualr controles.
- **latlon2xy.m**: Convierte coordenadas latitud-longitud a coordenadas cartesianas.
- **atan2c.m**: Implementación personalizada de la función `atan2`.
- **ekfMemNic.m**: Implementación de un Filtro de Kalman Extendido (EKF).
- **mheMemNic.m**: Implementación de un Estimador de Estado Basado en Memoria (MHE).
- **mheOptiLoss3.m**: Variante del MHE con optimización de pérdida.
- **mpc_RUN.m**: Ejecuta el control predictivo basado en modelo (MPC).

### Archivos de Código Compilado (`.mexa64`, `.mexw64`)
- **mpc_RUN.mexa64**: Versión compilada de `mpc_RUN.m` para Linux.
- **acado_MPCstep.mexa64**: Función optimizada de ACADO Toolkit para MPC.
- **f.mexa64 / f.mexw64**: Archivos compilados auxiliares.

### Archivos en C (`.c`)
- **f.c**: Código fuente en C relacionado con la funcion de optimizacion para ajustar offsets de los GPS.

## Requisitos
- MATLAB con soporte para archivos `.mexa64` o `.mexw64`, según el sistema operativo.
- CasADi para matlab incluido en el PATH.
- ACADO Toolkit si se usa para MPC.

## Uso
1. Extraer los archivos bags .zip.
2. Ejecutar `bag_plot_extensa.m`.
3. Ejecutar `bag_plot_ereducida.m`.
4. Ejecutar  `main_memoria.m`.

