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

## Análisis:

- Sabiendo que el robot Phantom X posee 4 GDL, de los cuales 3 corresponden a posición, el GDL restante
proporciona una medida independiente para un ángulo de orientación (asuma orientación en ángulos fijos).
¿De qué ángulo de orientación se trata?

**RESPUESTA:**

El robot Phantom X posee 4 GDL, como se indicó 3 de estos grados corresponden a movimientos en los 3 ejes principales x, y, z. El otro grado de libertad corresponde a un giro alrededor del eje y, que en una orientación de ángulos fijos es el ángulo pitch. Dada la disposición de las articulaciones y en especial la de la muñeca, este es el ángulo que robot puede lograr facilmente con el movimiento de una única articulación, para los otros ángulos el robot deberá moverse en los 3 ejes para lograr estas posiciones.

- ¿Cuántas soluciones posibles existen para la cinemática inversa del manipulador Phantom X ?

**RESPUESTA:**

Es posible lograr 2 soluciones al desarrollar la cinemática inversa, estas serían codo arriba y codo abajo. Matemáticamente, estas soluciones se evidencian por la dualidad que resulta al resolver una ecuación cuadrática, dando una solución con signo positivo y otra con signo negativo.

- Consulte en qué consiste el espacio diestro de un manipulador.

**RESPUESTA:**

Son el conjunto de puntos a los cuales el manipulador es capaz de llegar o alcanzar con al menos una orientación arbitraría, este espacio difiere del espacio de trabajo, el cual define al conjunto de puntos que alcanza el manupulador.

## ROS - Aplicación de Pick and place:

Se tiene como objetivo implementar una aplicación de Pick And Place con el robot Phantom X, que consiste
en lo siguiente:

1. El robot deberá tomar la pieza tipo 1 que se encuentre a su derecha y ubicarla al frente.

2. El robot deberá tomar la pieza tipo 2 que se encuentra a su izquierda, e insertarla en la pieza 1 que está
ahora al frente

Tomando como base el trabajo realizado en el laboratorio de cinemática directa, y utilizando el modelo de
cinemática inversa desarrollado, cree un script que ejecute la rutina descrita. Puede realizarlo por medio de
MATLAB o Python, según sea su preferencia.

**RESPUESTA:**

Para la implementación de este item, se desarrollaron los scripts adjuntos: "Segundo_punto.m", "movePX.m", "mapfun.m", "InverseKinematics.m", los cuales se describirán a continuación:

- El programa principal es el script "Segundo_punto.m", desde el cual se llaman las otras funciones. Este script principal consiste en la creación de un cliente de pose y posición, y también la creación de un mensaje, esto para la comunicación con el nodo de Ros. Y de la misma manera que se realizó el laboratorio anterior, se crea el robor con los parámetros DH. Luego, se construye la matriz T0 que es la matriz homogénea de la herramienta respecto a la base: 

```
cliente = rossvcclient('/dynamixel_workbench/dynamixel_command'); %Creación de cliente de pose y posición

msg = rosmessage(cliente); %Creación de mensaje

l = [13.27 10.3 10.3 6.57]; % Longitudes eslabones

l_T = 5.5;
% Definicion del robot RTB
L(1) = Link('revolute','alpha',-pi/2,'a',0,      'd',l(1),'offset',0);
L(2) = Link('revolute','alpha',0,    'a',l(2),   'd',0,   'offset',-pi/2,'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,    'a',l(3),   'd',0,   'offset',0,    'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,    'a',l(4),   'd',0,   'offset',0);

Robot = SerialLink(L,'name','Px');

Robot.tool = [0 0 1 0;
              1 0 0 0;
              0 1 0 0;
              0 0 0 1];


T0 =   Robot.fkine([0 0 0 0]);
```
Luego, se establecen las poses iniciales y finales que tendrá el TCP para lograr la tarea pick and place:

```
T1 = transl(2.5,-10,-sum(l) + l_T - 1)*T0*troty(pi); %Left
Tt1 = transl(0,0,sum(l)/2 - l_T)*T1;

TFt = transl(14.5,-0.8,-sum(l) + 2*l_T - 1)*T0*troty(pi);

T2 = transl(14.5,-0.8,-sum(l) + l_T)*T0*troty(pi); %Front
Tt2 = transl(0,0,sum(l)/2 - l_T)*T2;


T3 = transl(3.5,10,-sum(l) + l_T - 1)*T0*troty(pi);%Right
Tt3 = transl(0,0,sum(l)/2 - l_T)*T3;

T4 = transl(14.5,-0.8,-sum(l) + l_T + 1)*T0*troty(pi); %Front
Tt4 = Tt2;

```

Se concatena un vector con las matrices que calcula al pasarle a la función ctraj las posiciones iniciales y finales establecidas anteriormente y de manera similar se crea un vector llamado "gripper" que almacena las posiciones del gripper en cada una de estas poses:

```
Mov = cat(3,  ctraj(T0,Tt1,n), ... 
              ctraj(Tt1,T1,n), ... %Take the base
              ctraj(T1,Tt1,n), ...
              ctraj(Tt1,Tt2,n), ...
              ctraj(Tt2,TFt,n), ...
              ctraj(TFt,T2,n), ... %Put the base
              ctraj(T2,Tt2,n), ...
              ctraj(Tt2,Tt3,n), ...
              ctraj(Tt3,T3,n), ... %Take the disk
              ctraj(T3,Tt3,n), ...
              ctraj(Tt3,Tt4,n), ...
              ctraj(Tt4,TFt,n), ... 
              ctraj(TFt,T4,n), ... %Put the disk
              ctraj(T4,Tt4,n));

o_gripper = [ 0 -0.9408 -0.9408 -0.9408 -0.9408 ...
              0 0 0 -0.9408 -0.9408 -0.9408 -0.9408 0 0];
```

Por último, se crea un ciclo for para cada una de las poses alcazadas con la función ctraj, en donde se calcula la cinemática inversa de cada pose, se le da la orden al manipulador de moverse y se grafica estas poses en MATLAB. Dentro de este ciclo for hay un condicional que considera si el residuo al dividir i y n es 0, esto con el fin de indentificar cuando se ha recorrido la última matriz del vector Mov, cuando esta condición se cumpla será momento de pasarle al programa la pose del gripper:

```
for i=1:14*n
   thetas = InverseKinematics(Robot,l,Mov(:,:,i));
   movePX(msg,cliente,thetas, false);
   Robot.plot(thetas(2,:),'notiles','noname')
   hold on;
   trplot(eye(4),'rgb','arrow','length',25,'frame','or')
   hold on
   plot3(Mov(1,4,i),Mov(2,4,i),Mov(3,4,i),'ro')
   hold on;

   if mod(i,n) == 0
       movePX(msg,cliente,o_gripper(i/n), true);
       pause(1);
   end

end

```

- En cuanto a la función movePX.m, es esta función la encargada de hacer mover el manipulador, para lo cual solicita un mensaje, un cliente, un vector con los valores articulares y una condición booleana que definirá si el movimiento es en el gripper o en el resto de las articulaciones del manupulador: 

```
function movePX(msg,cliente,thetas,mov_gripper)

    if mov_gripper
        msg.AddrName = "Goal_Position";
        msg.Id = 5;
        msg.Value = mapfun(rad2deg(thetas),-150,150,0,1023);
        call(cliente,msg);
    else
        msg.AddrName = "Goal_Position";
        pause(0.1);
        q = rad2deg(thetas(2,:));
    %     q(5) = o_gripper;
    for i=4:-1:1
        msg.Id = i;
        msg.Value = mapfun(q(i),-150,150,0,1023);
        call(cliente,msg);
    end

    end
    

end
```
- Finalmente, en la función InverseKinematics.m se calculan los ángulos articulares mediante cinemática inversa, ingresando a la función las longitudes de los eslabones y la matriz de la TCP en la pose deseada. En este script se plantean las ecuaciones descritas en el primer item de este laboratorio, en donde se resulven para las dos poses posibles (codo arriba y codo abajo), estos ángulos articulares de las dos poses posibles son la salida de esta funciòn:

```
function thetas = InverseKinematics(Robot,l,Pose)
 
    HT0 = Pose; %Robot.fkine(q);
    
    d_T = HT0(1:3,4);
    approach = HT0(1:3,3);
    W = d_T - l(4)*approach;
    
    x_p = HT0(1,4);
    y_p = HT0(2,4);
    
    th1 = atan2(y_p,x_p);

    if abs(th1)>pi/2
        th1_m = atan2(-y_p,-x_p);
    else
        th1_m = th1;
    end
    x_trian = sqrt( W(1)^2 + W(2)^2);
    y_trian = W(3) - l(1);
    
    cos_th3 = (x_trian^2 + y_trian^2 - l(2)^2 - l(3)^2)/(2*l(2)*l(3));
    
    sin_th3 = sqrt(1-cos_th3^2); %Codo arriba (Es contrario a lo visto en clase porque acá el movto positivo es en el sentido del reloj)
    
    th3_up = atan2(real(sin_th3),cos_th3);
    
    th3_do = -th3_up; %Codo abajo
    
    alpha = atan2(-y_trian,x_trian); %Acá y se toma negativa puesto que el mvto de las juntas es contrario al sentido de la convención 
    beta= atan2(-l(3)*sin(abs(th3_do)),l(2)+l(3)*cos(abs(th3_do)));
    

    th2_do = alpha - beta; %Codo abajo
    th2_do = (pi/2+th2_do);
    
    th2_up = alpha + beta; %Codo arriba
    th2_up = (pi/2+th2_up);
    
    
    H30_do = Robot.A([1 2 3],[th1_m th2_do th3_do]);
    
    HT3_do = H30_do^(-1)*HT0;
    
    th4_do = atan2(-HT3_do(1,1),HT3_do(2,1));


    H30_up = Robot.A([1 2 3],[th1_m th2_up th3_up]);

    HT3_up = H30_up^(-1)*HT0;
    
    th4_up = atan2(-HT3_up(1,1),HT3_up(2,1));
    
    thetas = [th1 th2_do th3_do th4_do; ...
              th1 th2_up th3_up th4_up];   

end
```
## ROS - Aplicación de movimiento en el espacio de la tarea:

- De manera similar al trabajo realizado en el laboratorio de cinemática directa, cree un script (en Matlab o
Python, según se prefiera) que permita mover el robot de manera escalada, con la salvedad de que no de
manera articular como anteriormente se hizo, sino de forma operacional, es decir, en el espacio de la tarea del
efector final del robot (en x, en y, en z, etc). El script debería seguir la siguiente lógica:

1. Se debe realizar el movimiento por pasos, es decir, se debe establecer un avance (en traslación o en
orientación, según sea el caso) para que el robot realice ese avance en el eje que se solicitó.

2. Se le debe indicar al programa qué tipo de movimiento se desea realizar. Se debe imprimir en consola el
nombre del movimiento operativo. El tipo de movimiento se cambia de la siguiente manera:

  - Los tipos de movimiento son los siguientes, junto con el nombre que se debe imprimir en pantalla
para cada uno:

    1. Traslación en X - trax
    
    2. Traslación en Y - tray
    
    3. Traslación en Z - traz
    
    4. Rotación sobre el eje O del TCP - rot
    
    - Con la tecla ’W’ se pasa al siguiente tipo de movimiento. Por ejemplo, si está en trax, pasaría a tray,
y así.
  
    - Con la tecla ’S’ se pasa al tipo de movimiento anterior. Por ejemplo, si est ́a en rot, pasaría a traz, y
así.

    - Se puede realizar de manera cíclica, es decir, que el siguiente tipo de movimiento a rot sea trax, y
que el anterior de trax sea rot.

  - Al pulsar la tecla ’D’, el efector final debería dar un salto igual al avance establecido por el equipo, en la
dirección del tipo de movimiento seleccionado y en sentido positivo.

  - Al pulsar la tecla ’A’, el efector final debería dar un salto igual al avance establecido por el equipo, en la
dirección del tipo de movimiento seleccionado y en sentido negativo.

  - Obtenga la visualización del manipulador en RViz, de tal manera que se evidencien todos los movimientos
realizados en el espacio de la tarea. Nota: Los movimientos en el espacio de la tarea siempre se realizan
con respecto al sistema de referencia de la base.

**RESPUESTA:**

Para la implementación de este item, se desarrollaron los scripts adjuntos: "Tercer_punto.m", "movePX.m", "mapfun.m", "InverseKinematics.m", se describirá el script "Tercer_punto.m" a continuación:

Inicialmente, se crea un cliente para pose y posición, también se crea un mensaje al cual se le modifica los torques. Se crea el manipulador en MATLAB con los parámetros DH calculados en el laboratorio anterior:

```
cliente = rossvcclient('/dynamixel_workbench/dynamixel_command'); %Creación de cliente de pose y posición
msg = rosmessage(cliente); %Creación de mensaje

msg.AddrName = "Torque_Limit";

values = [800 800 400 400];

for i=1:4
        msg.Id = i+5;
        msg.Value = values(i);
        call(cliente,msg);
%         pause(0.5);
end


l = [3.9 10.57 10.6 6.57]; % Longitudes eslabones
% Definicion del robot RTB
L(1) = Link('revolute','alpha',-pi/2,'a',0,      'd',l(1),'offset',0);
L(2) = Link('revolute','alpha',0,    'a',l(2),   'd',0,   'offset',-pi/2,'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,    'a',l(3),   'd',0,   'offset',0,    'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,    'a',l(4),   'd',0,   'offset',0);

Robot = SerialLink(L,'name','Px');

Robot.tool = [0 0 1 0;
              1 0 0 0;
              0 1 0 0;
              0 0 0 1];

```
Se calcula la MTH del manipulador en la posición home que definimos, en la práctica esta posición es con el manipulador totalmente vertical. Luego se definen la cantidad de poses que se quieren entre una posición inicial y final con la variable n, y la distacia que se quiere recorrer entre pose y pose es de 1 cm. Además, se crean las variables booleanas que indicaran si el manipulador debe moverse porque se completaron los cálculos y otra variable que indicará si el movimiento se hace respecto al marco de refencia de la base o con respecto al marco de la herramienta:
```
move = 0;
T0 =  Robot.fkine([1.5544 0.7056 1.9226 0.3988]); %Robot.fkine([0 0 0 0]);
n = 2; %Number of poses
d = 1;

should_move = false;
move_with_T = false;
```
Luego, se entrará a un ciclo infinito que imprimirá en pantalla el movimiento actual y empezará a preguntar por los movimientos deseados, si la variable move_with_T es falsa el cálculo se hará con respecto a la base y por el contrario, si es verdadero el cálculo se hará respecto a la herramienta, esta selección que se hace desde el código se mostrará en pantalla también:
```
 movements = ["trax" "tray" "traz" "rot"];

    str = sprintf('ACTUAL MOVEMENT: %s \n',movements(mod(move,4)+1));

    prompt = "What do you want to do? \n \n W: Next movement type \n " + ...
        "S: Previous movement type \n D: Move in positive direction  \n " + ...
        "A: Move in negative direction \n \n" + str;

    if move_with_T
        prompt = prompt + "  respect the tool";
    else
        prompt = prompt + "  respect the base";
    end
```
En caso que la letra seleccionada sea la W, se le sumará 1 a la variable move, que finalmente representa el id de cada articulación. Por otro lado, en caso que la letra oprimida sea S, se le restará 1 a esta variable move:

```
case 'w'
            move = move + 1;
        case 's'
            if move > 0
                move = move - 1;
            end
```
Si la tecla oprimida es A y si la variable move_with_T es verdadera se asignarán el punto inicial de movimiento como la posición home definida anteriormente y la posición final como el desplazamiento d en el eje seleccionado y en el sentido seleccionado. AQUI VOY

## Análisis:

solución codo arriba
Siguiento la plantilla creada (Ver imagen a continuación), se establecieron las poses iniciales y finales que queríamos que el manipulador alcanzara 
numero de poses, delay

torques
premultiplicacion
## Conclusiones:

## Referencias:
