# Estimación de Estados en Vehículos Terrestres

Este repositorio contiene el código fuente asociado a la memoria de título:

**“Estimación de estados de vehículos terrestres usando estimación de horizonte móvil con mediciones redundantes de GPS de bajo coste, incorporando restricciones geométricas”**

Desarrollada por **Nicolás Mansilla** para optar al título de **Ingeniero Civil Electrónico** en la **Universidad Técnica Federico Santa María**.

---

## Descripción General

El objetivo de este trabajo es diseñar, simular y evaluar algoritmos de estimación de estado para plataformas móviles terrestres, integrando múltiples mediciones de GPS de bajo costo.  
Se propone una arquitectura basada en un Estimador de Horizonte Móvil (MHE), combinada con un Filtro de Kalman Extendido (EKF) y estrategias de Control Predictivo (MPC) para validación experimental.  
Además, se incorporan restricciones geométricas derivadas de la cinemática del vehículo para mejorar la precisión y robustez ante errores de medición.

---

## Estructura del Repositorio

### Archivos MATLAB (`.m`)

| Archivo               | Descripción                                                                 |
|-----------------------|-----------------------------------------------------------------------------|
| `main_memoria.m`      | Script principal para estimación, simulación y control.                     |
| `latlon2xy.m`         | Conversión de coordenadas geodésicas (lat/lon) a plano cartesiano local.    |
| `atan2c.m`            | Implementación personalizada de `atan2` para continuidad angular.            |
| `ekfMemNic.m`         | Implementación del Filtro de Kalman Extendido (EKF).                         |
| `mheMemNic.m`         | Implementación del Estimador de Horizonte Móvil (MHE).                       |
| `mheOptiLoss3.m`      | Variante del MHE con función de pérdida optimizada.                          |
| `mpc_RUN.m`           | Ejecución del Control Predictivo Basado en Modelo (MPC).                     |
| `bag_plot_extensa.m`  | Visualización completa de datos `.bag`.                                     |
| `bag_plot_ereducida.m`| Visualización resumida de datos `.bag`.                                     |

### Archivos Compilados (`.mexa64`, `.mexw64`)

| Archivo                    | Plataforma | Descripción                                                 |
|----------------------------|------------|-------------------------------------------------------------|
| `mpc_RUN.mexa64`           | Linux      | Versión compilada del controlador MPC.                      |
| `acado_MPCstep.mexa64`     | Linux      | Paso optimizado del MPC generado con ACADO Toolkit.         |
| `f.mexa64`, `f.mexw64`     | Linux/Win  | Funciones auxiliares para optimización GPS vía `mex`.       |

### Archivos en C (`.c`)

| Archivo | Descripción                                                                 |
|---------|-----------------------------------------------------------------------------|
| `f.c`   | Función en C para optimización de offsets GPS, integrada con MHE vía MEX.   |

---

## Requisitos

- MATLAB R2021a o superior.
- Compatibilidad con archivos `.mexw64` (Windows) o `.mexa64` (Linux).
- [CasADi](https://web.casadi.org/) para MATLAB (agregado al `PATH`).
- [ACADO Toolkit](https://acado.github.io/) (solo si se usa MPC).

---

## Instrucciones de Ejecución

```bash
# Clona el repositorio
git clone https://github.com/Nico-Mansilla/memoria.git
cd memoria
```

Luego, en MATLAB:

1. Extraer los archivos `.zip` que contienen datos `.bag`.
2. Visualizar los datos:
   ```matlab
   bag_plot_extensa
   bag_plot_ereducida
   ```
3. Ejecutar el script principal:
   ```matlab
   main_memoria
   ```

---

## Notas

- Todo el código es ejecutable directamente desde MATLAB (no requiere Simulink).
- Los scripts están estructurados para facilitar cambios en sensores, modelos o controladores.
- Incluye comentarios extensos para comprensión y modificación por terceros.

---

## Autor

**Nicolás Mansilla**  
Ingeniería Civil Electrónica  
Universidad Técnica Federico Santa María  
https://github.com/Nico-Mansilla

---

## Licencia

Este código se encuentra disponible únicamente con fines académicos y de investigación. Para usos comerciales, contactar al autor.
