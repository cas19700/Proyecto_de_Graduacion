%% Simulación de Boids de Reynolds
% Compu path: C:\Users\Brayan\OneDrive - MABREX CENTROAMERICA S.A\Documentos\Año 2023\Segundo semestre\Tesis\PruebasFisicas
%% Implementación Física variables
clear all;
%tStart = tic;

robot = robotat_connect();
pol(1) = robotat_3pi_connect(2)
%pol(2) = robotat_3pi_connect(3) % 
%pol(3) = robotat_3pi_connect(4)

MAX_WHEEL_VELOCITY = 850; % Velocidad máxima ruedas (en rpm) 
MAX_WHEEL_VELOCITY = 850*(2*pi/60); % Velocidad máxima ruedas (en rad/s)
WHEEL_RADIUS = (32/2)/1000; % Radio de las ruedas (en m)
DISTANCE_FROM_CENTER = (94/2)/1000; % Distancia a ruedas (en cm)
MAX_SPEED = WHEEL_RADIUS * MAX_WHEEL_VELOCITY;% Velocidad lineal máxima (en cm/s)

%% Pruebas Pol4
% Usado para comprobar que los Pololu funcionen adecuadamente
robotat_3pi_set_wheel_velocities(pol(1),100,-100)
robotat_3pi_force_stop(pol(1))


robotat_3pi_set_wheel_velocities(pol(2),100,-100)
robotat_3pi_force_stop(pol(2))


robotat_3pi_set_wheel_velocities(pol(3),100,-100)
robotat_3pi_force_stop(pol(3))


%% Obtener posiciones de los Pololu

% Pololu #3 CAMBIAR A 3, ESTA EN 6 CAMBIAR ESTE SIEMPRE EN EL FLOCK TAMBIEN
pos1 = robotat_get_pose(robot, 2, 'eulxyz') % Obtener la posicion del pololu y angulos
bearing1 = pos1(6)+136.5751;                 % Ajuste para el angulo de bearing
bearing1 = deg2rad(bearing1);                % Cambio a radianes

% Pololu #4   CAMBIAR A 4 ESTA EN 8(HOY ESTABAN USANDOLO)
pos2 = robotat_get_pose(robot, 3, 'eulxyz') % Obtener la posicion del pololu y angulos
bearing2 = pos2(6)-179.8062;                 % Ajuste para el angulo de bearing
bearing2 = deg2rad(bearing2);                % Cambio a radianes

% Pololu #5
pos3 = robotat_get_pose(robot, 4, 'eulxyz') % Obtener la posicion del pololu y angulos
bearing3 = pos3(6)-132.0875;                 % Ajuste para el angulo de bearing
bearing3 = deg2rad(bearing3);                % Cambio a radianes

%% Simulación
boids_count=1; % Numero de pololus a utilizar
predator_count=1;
predator=Predator.empty;
boids=Boid.empty; % Espacios
predator=Predator(200,200)
% Pololu #3
boids(1) = Boid(pos1(1)*100+380/2,pos1(2)*100+480/2,bearing1,predator) % Envios de posiciones, angulo de bearing y desfases (380/2)
                                                                 % El +10 tambien es para un desfase y hacer 
                                                                 % mas pequeño el mapa                                                          
%% Pololu #4
boids(2) = Boid(pos2(1)*100+380/2,pos2(2)*100+480/2,bearing2,predator) % Envios de posiciones, angulo de bearing y desfases (380/2)
                                                                 % El +10 tambien es para un desfase y hacer 
                                                                 % mas pequeño el mapa
%% Pololu #5                                                                 
boids(3) = Boid(pos3(1)*100+380/2,pos3(2)*100+480/2,bearing3,predator) % Envios de posiciones, angulo de bearing y desfases (380/2)
                                                                 % El +10 tambien es para un desfase y hacer 
                                                                 % mas pequeño el mapa                                                                
%% Correr la simulación                                                                
tic
%pol = pol
flock=Flock(boids,[380 480], pol,robot,predator);% Se cargan las dimensiones de los bordes del programa (x=370, y=470)
trad = flock.ftimes % Esto creo que ya no lo use
fprintf('Tiempos:'); % Esto tampoco
f = figure;
plane = Plane(f,[380 480],boids,robot,predator) % Se visualiza la simulación
flocksim = flock.run(plane);   % Se empiezan a correr los renderizados
%% Controlador y Envio de datos
% PID posición
kpP = 1;
kiP = 0.0001; 
kdP = 0.5;
EP = 0;
eP_1 = 0;

% PID orientación
kpO = 2*5;
kiO = 0.0001; 
kdO = 0;
EO = 0;
eO_1 = 0;

% Acercamiento exponencial
%v0 = 1000;
%alpha = 20;
%v0 = 80;
%alpha = 0.9;
v0 = 0.5;
alpha = 0.7;
nval = 1;
% Arreglos para generar los espacios de memoria
cor = zeros(1,2);
xf = zeros(1,2);
yf = zeros(1,2);
xf(1,1) = pos1(1)*100+380/2; % Ajustamos el eje x para hacer que concuerde con la plataforma
yf(1,1) = pos1(2)*100+480/2;
wctrl = flocksim.arr_sp;
vctrl = boids.max_speed;
% Puntos a alcanzar
xgoal = flocksim.xvals 
ygoal = flocksim.yvals
% Suavizado
cpts = [xgoal(1,:);ygoal(1,:)]
tpts = [0 25];
tvect = 0:5:25
%tpts = [0 150];
%tvect = 0:1:150

[q, qd, qdd, pp] = bsplinepolytraj(cpts,tpts,tvect); % Se obtienen valores mas suavizados
figure
plot(cpts(1,:),cpts(2,:),'xb-') % Se grafican los valores originales
hold all
plot(q(1,:), q(2,:)) % Se grafican los nuevos valores
xlabel('X')
ylabel('Y')
hold off
%%
reff=0
bear_m = [0,136.5751,-179.8062,-132.0875] % angulos de bearing obtenidos anteriormente
tam1 = length(q);
k=1;
detener = zeros(1,length(boids_count));
        while(1)%for k=1:tam1    
            for i=1:1 %boids_count
                roba = robotat_get_pose(robot, 2, 'eulxyz'); % i + 1, Obtener la posicion actual
                bearing = roba(6)+bear_m(2); % Ajustar el angulo de bearing
                bearing = deg2rad(bearing); % Grados a radianes
                x = roba(1)*100+380/2; % Ajustamos el eje x para hacer que concuerde con la plataforma
                y = roba(2)*100+480/2; % Ajustamos el eje y para hacer que concuerde con la plataforma
                theta = bearing;   % theta igual al angulo de bearing
                ref = [q(1,nval); q(2,nval)]; % Referencia igual a los valores obtenidos anteriormente
                e = [ref(1) - x; ref(2) - y]; % Calculo del error
                thetag = atan2(e(2), e(1)); % Diferencia enter angulo deseado
                av =  [x; y]; % Actual Value
                eP = norm(e); % Error de posicion
                eO = thetag - theta; % Error de orientacion
                eO = atan2(sin(eO), cos(eO)); 

                % Control de velocidad lineal
                kP = v0 * (1-exp(-alpha*eP^2)) / eP;
                v = kP*eP;

                % Control de velocidad angular
                eO_D = eO - eO_1;
                EO = EO + eO;
                w = kpO*eO + kiO*EO + kdO*eO_D;
                eO_1 = eO;

                % Se combinan los controladores
                u = [v; w];
                xv = abs(ref(1))-abs(x)
                yv = abs(ref(2))-abs(y)  
                refff(1,k) = abs(xv)
                refff(2,k) = abs(yv)
                if (abs(xv)<=5) % Comprobamos que este a 10 cm por lo menos para el cambio de punto
                      
                   reff(1,k) = xv
                   %reff(2,k) = yv 
                   reff(3,k) = abs(xv)
                   %reff(4,k) = abs(yv)
                   %nval = nval+1 % Aumentar el contador
                   
                   if (abs(yv)<=7)
                       reff(2,k) = yv 
                       reff(4,k) = abs(yv)
                       nval = nval+1 % Aumentar el contador
                       pose = robotat_get_pose(robot, 2, 'eulxyz');
                       xf(nval) = pose(1)*100+380/2; % Guardar valores de x
                       yf(nval) = pose(2)*100+480/2; % Guardar valores de y
                       
                   end
                       
                       if (nval > tam1)
                           nval = tam1;
                           robotat_3pi_force_stop(pol(i)) % Si llega al ultimo valor detenerse
                           %detener(1,i) = 1;
                           detener = 1;
                           break;
                       end
                end
                
                  
                wr = (u(1)+DISTANCE_FROM_CENTER*u(2))/WHEEL_RADIUS % Velocidad de la llanta derecha
                wl = (u(1)-DISTANCE_FROM_CENTER*u(2))/WHEEL_RADIUS % Velocidad de la llanta izquierda

                if wr > 300
                    wr = 100; % Limitamos la velocidad a 300rpm
                end


                if wl > 300
                    wl = 100; % Limitamos la velocidad a 300rpm
                end


                if wr < -300
                    wr = -100; % Limitamos la velocidad a 300rpm
                end


                if wl < -300 
                    wl = -100; % Limitamos la velocidad a 300rpm
                end           
                
                if abs(wr) < 30
                    %wr = wr+20; % Si la velocidad es muy poca ajustarla en 30 rpm
                end
                robotat_3pi_set_wheel_velocities(pol(i), wl, wr);% Envio de datos de rueda derecha e izquierda
                k = k + 1;
                yv = 100;
                xv = 100;
                if detener == 1 
                    break;
                end    
            end
        end
        
        
robotat_3pi_force_stop(pol(1)) % Detiene al pololu 3
%robotat_3pi_force_stop(pol(2)) % Detiene al pololu 4
%robotat_3pi_force_stop(pol(3)) % Detiene al pololu 4
% bsplinepolytraj
%% Gráficas
figure(18)
plot(q(1,:), q(2,:)) % Valores de la simulacion interpolados
%axis([min(flocksim.xvals), max(flocksim.xvals), min(flocksim.yvals), max(flocksim.yvals)]);
hold on;
xf = xf(1,2:7)
yf = yf(1,2:7)
plot(xf,yf) % Valores obtenidos en la implementación fisica
% RECORDAR ACTIVAR LAS COR EN FLOCK

%Error cuadratico medio
%%
ECMx = sum((xf-q(1,:)))/(length(xf)) % Calculo del error en x
ECMy = sum((yf-q(2,:)))/(length(yf)) % Calculo del error en y

ECMT = sqrt(ECMx^2+ECMy^2) % Calculo del error total

err = immse([q(1,:);q(2,:)], [xf;yf]); % Error cuadratico medio
fprintf('\n El error cuadratico medio es %0.4f\n', err);

%% Parar el robot en caso de emergencia
robotat_3pi_force_stop(pol(1)) % Detiene al pololu 3
robotat_3pi_force_stop(pol(2)) % Detiene al pololu 4
robotat_3pi_force_stop(pol(3)) % Detiene al pololu 5

%% Desconexion del robot
robotat_3pi_disconnect(2)
robotat_3pi_disconnect(3)
robotat_3pi_disconnect(4) %cambiar al 4
%robotat_3pi_disconnect(5)
robotat_disconnect(robot)

% PROBAR LO DE QUE TAL SIGUEN LOS ROBOTS LA SIMULACION SOLO CORRER ESTO y
% mirar
