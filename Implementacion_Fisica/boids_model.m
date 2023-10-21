%% Simulación de Boids de Reynolds

%% Implementación Física variables
clear all;
%tStart = tic;

robot = robotat_connect();
pol(1) = robotat_3pi_connect(2)
%pol(2) = robotat_3pi_connect(3) % 
%pol(3) = robotat_3pi_connect(4)

MAX_WHEEL_VELOCITY = 850; % velocidad máxima ruedas (en rpm) 
MAX_WHEEL_VELOCITY = 850*(2*pi/60); % velocidad máxima ruedas (en rad/s)
WHEEL_RADIUS = (32/2)/1000; % radio de las ruedas (en m)
DISTANCE_FROM_CENTER = (94/2)/1000; % distancia a ruedas (en cm)
% Velocidad lineal máxima (en cm/s)
MAX_SPEED = WHEEL_RADIUS * MAX_WHEEL_VELOCITY;

%% Obtener posiciones de los Pololu

% Pololu #3 CAMBIAR A 3, ESTA EN 6 CAMBIAR ESTE SIEMPRE EN EL FLOCK TAMBIEN
pos1 = robotat_get_pose(robot, 2, 'eulxyz') % Obtener la posicion del pololu y angulos
bearing1 = pos1(6)+133.8023;                 % Ajuste para el angulo de bearing
bearing1 = deg2rad(bearing1);                % Cambio a radianes

% Pololu #4   CAMBIAR A 4 ESTA EN 8(HOY ESTABAN USANDOLO)
pos2 = robotat_get_pose(robot, 3, 'eulxyz') % Obtener la posicion del pololu y angulos
bearing2 = pos2(6)+179.6529;                 % Ajuste para el angulo de bearing
bearing2 = deg2rad(bearing2);                % Cambio a radianes

% Pololu #5
pos3 = robotat_get_pose(robot, 4, 'eulxyz') % Obtener la posicion del pololu y angulos
bearing3 = pos3(6)-132.0875;                 % Ajuste para el angulo de bearing
bearing3 = deg2rad(bearing3);                % Cambio a radianes

%% Simulación
boids_count=1; % Numero de pololus a utilizar
boids=Boid.empty; % Espacios
%for i=1:boids_count
%   boids(i)=Boid(rand*380,rand*480);
%end

% Pololu #3
boids(1) = Boid(pos1(1)*100+380/2+10,pos1(2)*100+480/2+10,bearing1) % Envios de posiciones, angulo de bearing y desfases (380/2)
                                                                 % El +10 tambien es para un desfase y hacer 
                                                                 % mas pequeño el mapa                                                          
%% Pololu #4
boids(2) = Boid(pos2(1)*100+380/2+10,pos2(2)*100+480/2+10,bearing2) % Envios de posiciones, angulo de bearing y desfases (380/2)
                                                                 % El +10 tambien es para un desfase y hacer 
                                                                 % mas pequeño el mapa
%% Pololu #5                                                                 
boids(3) = Boid(pos3(1)*100+380/2+10,pos3(2)*100+480/2+10,bearing3) % Envios de posiciones, angulo de bearing y desfases (380/2)
                                                                 % El +10 tambien es para un desfase y hacer 
                                                                 % mas pequeño el mapa                                                                

%% Pruebas Pol4

robotat_3pi_set_wheel_velocities(pol(1),100,-100)
robotat_3pi_force_stop(pol(1))


robotat_3pi_set_wheel_velocities(pol(2),100,-100)
robotat_3pi_force_stop(pol(2))


robotat_3pi_set_wheel_velocities(pol(3),100,-100)
robotat_3pi_force_stop(pol(3))

%% Correr la simulación                                                                
tic
%pol = pol
flock=Flock(boids,[370 470], pol,robot);% Se cargan las dimensiones de los bordes del programa (x=370, y=470)
trad = flock.ftimes % Esto creo que ya no lo use
fprintf('Tiempos:'); % Esto tampoco
            
%tEnd = toc(tStart)
%ftime=toc
%fprintf('Tiempo transcurrido: %.4f segundos\n', ftime);
%boids = boids
f = figure;
plane = Plane(f,[380 480],boids,robot) % Se visualiza la simulación
flocksim = flock.run(plane);   % Se empiezan a correr los renderizados
%% Controlador y Envio de datos
           
%% Controlador
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
v0 = 20;
alpha = 0.7;
nval = 1;
% Arreglos para generar los espacios de memoria
cor = zeros(1,151);
xf = zeros(1,151);
yf = zeros(1,151);
wctrl = flocksim.arr_sp;
vctrl = boids.max_speed;
% Puntos a alcanzar
xgoal = flocksim.xvals 
ygoal = flocksim.yvals
% Suavizado
cpts = [xgoal;ygoal]
tpts = [0 150];
tvect = 0:5:150
%tpts = [0 150];
%tvect = 0:1:150

[q, qd, qdd, pp] = bsplinepolytraj(cpts,tpts,tvect);
figure
plot(cpts(1,:),cpts(2,:),'xb-')
hold all
plot(q(1,:), q(2,:))
xlabel('X')
ylabel('Y')
hold off
%%
bear_m = [0,133.9023,179.6529,-132.0875] % angulos de bearing obtenidos anteriormente
tam1 = length(q);
        for k=1:tam1    
            for i=1:1 %boids_count
                roba = robotat_get_pose(robot, 2, 'eulxyz'); % i + 1
                bearing = roba(6)+bear_m(2);
                bearing = deg2rad(bearing);
                x = roba(1)*100+380/2+10; 
                y = roba(2)*100+480/2+10; 
                theta = bearing;   
                ref = [q(1,nval); q(2,nval)];
                e = [ref(1) - x; ref(2) - y];
                thetag = atan2(e(2), e(1));
                av =  [x; y];
                eP = norm(e);
                eO = thetag - theta;
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
                
                  if (((abs(ref(1))-abs(av(1)))<=10)&&((abs(ref(2))-abs(av(2)))<=10))
                        nval = nval+1
                            if (nval > tam1)
                                nval = tam1;
                                robotat_3pi_force_stop(pol(i))
                                break;
                            end
                  end
                
                wr = (u(1)+DISTANCE_FROM_CENTER*u(2))/WHEEL_RADIUS
                wl = (u(1)-DISTANCE_FROM_CENTER*u(2))/WHEEL_RADIUS
                
                %wl = (vctrl-DISTANCE_FROM_CENTER*wctrl(i,k))/WHEEL_RADIUS;
                % Los valores se envian en rpm           


                if wr > 300
                    wr = 300;
                end


                if wl > 300
                    wl = 300;
                end


                if wr < -300
                    wr = -300;
                end


                if wl < -300
                    wl = -300;
                end           
                
                if abs(wr) < 30
                    wr = wr+20;
                end
                robotat_3pi_set_wheel_velocities(pol(i), wl, wr);% Envio de datos de rueda derecha e izquierda
                pose = robotat_get_pose(robot, 2, 'eulxyz');
                xf(k) = pose(1)*100+380/2+10;
                yf(k) = pose(2)*100+480/2+10;
            end
        end
        
        
robotat_3pi_force_stop(pol(1)) % Detiene al pololu 3
robotat_3pi_force_stop(pol(2)) % Detiene al pololu 4
robotat_3pi_force_stop(pol(3)) % Detiene al pololu 4
% bsplinepolytraj
%% Gráficas

plot(flocksim.xvals,flocksim.yvals)
%axis([min(flocksim.xvals), max(flocksim.xvals), min(flocksim.yvals), max(flocksim.yvals)]);
hold on;

plot(xf,yf)
% RECORDAR ACTIVAR LAS COR EN FLOCK

%Error cuadratico medio

ECMx = sum((xf-flocksim.xvals))/(length(xf))
ECMy = sum((yf-flocksim.yvals))/(length(yf))

ECMT = sqrt(ECMx^2+ECMy^2)

err = immse([flocksim.xvals;flocksim.yvals], [xf;yf]);
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
