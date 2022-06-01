# Lab3_robotica
# Estudiantes: Alejandra Rojas - Juan Diego Plaza
## Ejercicios en el laboratorio:
***
## MATLAB + toolbox:

- Siguiendo la metodología analizada en clase desarrolle un modelo de cinemática inversa del manipulador
Phantom X e impleméntelo en MATLAB. Se recomienda usar el toolbox para verificar la solución hallada.

**RESPUESTA:**

- Esboce el espacio de trabajo del robot Phantom X.

**RESPUESTA:**

- Consulte los métodos disponibles en el toolbox para determinar la cinemática inversa de un manipulador.

**RESPUESTA:**

## Anàlisis:

- Sabiendo que el robot Phantom X posee 4 GDL, de los cuales 3 corresponden a posición, el GDL restante
proporciona una medida independiente para un ángulo de orientación (asuma orientación en ángulos fijos).
¿De qué ángulo de orientación se trata?

**RESPUESTA:**

- ¿Cuántas soluciones posibles existen para la cinemática inversa del manipulador Phantom X ?

**RESPUESTA:**

- Consulte en qué consiste el espacio diestro de un manipulador.

**RESPUESTA:**

## ROS - Aplicación de Pick and place:

Se tiene como objetivo implementar una aplicación de Pick And Place con el robot Phantom X, que consiste
en lo siguiente:

1. El robot deberá tomar la pieza tipo 1 que se encuentre a su derecha y ubicarla al frente.
2. El robot deberá tomar la pieza tipo 2 que se encuentra a su izquierda, e insertarla en la pieza 1 que está
ahora al frente

Tomando como base el trabajo realizado en el laboratorio de cinemática directa, y utilizando el modelo de
cinemática inversa desarrollado, cree un script que ejecute la rutina descrita. Puede realizarlo por medio de
MATLAB o Python, según sea su preferencia.
