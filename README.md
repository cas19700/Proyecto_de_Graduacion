# Proyecto de Graduacion: Implementación del algoritmo de Boids de Reynolds en robots móviles con ruedas dentro del ecosistema Robotat
## Simulación del algoritmo de Boids de Reynolds implementado en Matlab
En la carpeta Simulación se encuentra unicamente el algoritmo de Boids de Reynolds funcionando en Matlab sin la implementación física. En este se encuentra el boids_model el cual es donde ejecutaremos el código,
siempre recordar que necesitamos tener los archivos dentro de la misma carpeta. Regresando al archivo boids_model, en este podemos definir el número de agentes a utilizar en la variable boids_count, donde se
generaran posiciones aleatorias para estos. Asimismo, también estan los depredadores, en este caso definido a uno solo donde el usuario puede ingresar manualmente su posición en un rango de [0, 380] para "x" y 
un rango de [0, 480] para "y". Luego de definir dichos parámetros solo es necesario ejecutar el archivo y se desplegará la simulación, para terminarla simplemente es necesario salir de la misma.

## Implementación física del algoritmo de Boids de Reynolds implementado en Matlab
En la carpeta Implementación_Física se encuentran todas las funciones para comunicarse con el ecosistema Robotat y los Pololu 3pi+. Además de esto se vuelven a encontrar los archivos de la simulación pero en este caso ya estan adaptados para funcionar con los Pololu 3pi+.
## Secciones
El programa cuenta con varias secciones simplemente es necesario ejecutarlas en orden. Sin embargo, a continuación se dara un poco más de detalle sobre que hace cada sección, aun así el archivo (boids_model) ya esta comentado.
### Variables Implementación Física
Aquí se definieron algunas variables como lo es pol, la cual es un array de todos los Pololu que esten disponibles en el momento. Además estan las constantes como el radio de la llanta de los Pololu y demás valores.
### Pruebas Robots Pololu 3pi+
Esta sección no es necesario ejecutarla pero sirve para confirmar que el ESP tiene la programación adecuada para comunicarse con el ecosistema Robotat.
### Obtener posiciones de los Pololu 3pi+
En este caso se utiliza la funcion robotat_get_pose para los agentes disponibles permitiendo obtener las posiciones y en este caso hacer ajustes para el ángulo de bearing.
### Simulación
Esta sección es similar a la mencionada anteriormente, la unica diferencia es que en este caso las posiciones de los agentes no seran aleatorias sino que estaran dadas por los markers.
### Ajuste de plano
En este se ajusta los valores que proporciona el ecosistema Robotat, dado que el origen se encuentra en el centro y no en la esquina inferior izquierda por lo que tomando las dimensiones de la plataforma se desfasa el origen para los valores obtenidos.
### Ejecutar simulación
En esta sección se ve una previsualización de la simulación la cual esta limitada a 500 ciclos, esta ayudara a saber a que posiciones iran cada agente.
### Controlador e inicialización de registro de datos
En esta parte encontramos los coeficientes utilizados para el controlador y los arreglos que se utilizaran para registrar las posiciones de los agentes en físico una vez hagan el recorrido.
### Puntos a alcanzar
Esta sección obtiene las posiciones de la simulación que seran utilizadas posteriormente para realizar las interpolaciones de cada trayectoria.
### Suavizados (Interpolación)
En esta parte se interpolan y grafican tanto la trayectoria original como la nueva trayectoria a seguir, dado que se separaron los puntos que da originalmente la simulación. Es bueno recordar que unicamente es necesario ejecutar las interpolaciones de los agentes que se tienen disponibles. En caso se tenga duda de la interpolación puede usar el help de Matlab para entender la función bsplinepolytraj.
### Envío y recepción de datos
En esta parte se ejecuta el ciclo while para el controlador y el for para cada agente este ira registrando los datos una vez se haya llegado a una tolerancia aceptable. Una vez los robots alcancen sus ultimas posiciones el programa quedara en el while por lo que sera necesario hacer la siguiente conbinación de teclas "Ctrl + C" para deterla, asegurarse que la variable nval haya llegado al máximo del numero de puntos interpolados.
### Gráficas
La sección gráficas compara los valores de las trayectorias interpoladas con las recorridas por los robots Pololu 3pi+. Asimismo se puede obtener el error cuadrático medio si asi lo desea.
### Parar el robot en caso de emergencia
Esta sección es si en dado caso surge un error y los agentes quedan dando vueltas con diferentes velocidades, tambien hay una sección como esta en envío y recepción de datos fuera del while.
### Desconexión del robot
En esta sección se realiza la desconexión del Robotat y los Pololus a los que se haya conectado. RECORDAR siempre guardar el workspace antes de ejecutar esta sección dado que hacer la desconexión limpia automáticamente el workspace.
