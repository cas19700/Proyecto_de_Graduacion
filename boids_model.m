%% Simulación de Boids de Reynolds

%% Implementación Física variables
clear all;
%tStart = tic;

robot = robotat_connect();
pol(1) = robotat_3pi_connect(3)
pol(2) = robotat_3pi_connect(4) % CAMBIAR A 4 (HOY ESTABAN USANDOLO)
%pol(3) = robotat_3pi_connect(5)

MAX_WHEEL_VELOCITY = 850; % velocidad máxima ruedas (en rpm) 
MAX_WHEEL_VELOCITY = 850*(2*pi/60); % velocidad máxima ruedas (en rad/s)
WHEEL_RADIUS = (32/2)/1000; % radio de las ruedas (en m)
DISTANCE_FROM_CENTER = (94/2)/1000; % distancia a ruedas (en cm)
% Velocidad lineal máxima (en cm/s)
MAX_SPEED = WHEEL_RADIUS * MAX_WHEEL_VELOCITY;

%% Obtener posiciones de los Pololu

% Pololu #3 CAMBIAR A 3, ESTA EN 6 CAMBIAR ESTE SIEMPRE EN EL FLOCK TAMBIEN
pos1 = robotat_get_pose(robot, 3, 'eulxyz') % Obtener la posicion del pololu y angulos
bearing1 = pos1(6)+175.4321;                 % Ajuste para el angulo de bearing
bearing1 = deg2rad(bearing1);                % Cambio a radianes

% Pololu #4   CAMBIAR A 4 ESTA EN 8(HOY ESTABAN USANDOLO)
pos2 = robotat_get_pose(robot, 4, 'eulxyz') % Obtener la posicion del pololu y angulos
bearing2 = pos2(6)-136.8702;                 % Ajuste para el angulo de bearing
bearing2 = deg2rad(bearing2);                % Cambio a radianes

% Pololu #5
pos3 = robotat_get_pose(robot, 5, 'eulxyz') % Obtener la posicion del pololu y angulos
bearing3 = pos3(6)+176.8690;                 % Ajuste para el angulo de bearing
bearing3 = deg2rad(bearing3);                % Cambio a radianes

%% Simulación
boids_count=2; % Numero de pololus a utilizar
boids=Boid.empty; % Espacios
%for i=1:boids_count
%   boids(i)=Boid(rand*380,rand*480);
%end

% Pololu #3
boids(1) = Boid(pos1(1)*100+380/2+10,pos1(2)*100+480/2+10,bearing1) % Envios de posiciones, angulo de bearing y desfases (380/2)
                                                                 % El +10 tambien es para un desfase y hacer 
                                                                 % mas pequeño el mapa                                                          
% Pololu #4
boids(2) = Boid(pos2(1)*100+380/2+10,pos2(2)*100+480/2+10,bearing2) % Envios de posiciones, angulo de bearing y desfases (380/2)
                                                                 % El +10 tambien es para un desfase y hacer 
                                                                 % mas pequeño el mapa
%% Pololu #5                                                                 
boids(3) = Boid(pos3(1)*100+380/2+10,pos3(2)*100+480/2+10,bearing3) % Envios de posiciones, angulo de bearing y desfases (380/2)
                                                                 % El +10 tambien es para un desfase y hacer 
                                                                 % mas pequeño el mapa                                                                

%% Pruebas Pol4

robotat_3pi_set_wheel_velocities(pol(2),100,-100)
robotat_3pi_force_stop(pol(2))

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
%% Envio de datos
           
            % FOR AQui
            %for i=1:1
                cor = zeros(1,151);
                xf = zeros(1,151);
                yf = zeros(1,151);
                wctrl = flocksim.arr_sp;
                vctrl = boids.max_speed;
        for k=1:length(flocksim.arr_sp)    
            for i=1:boids_count
                wr = (vctrl+DISTANCE_FROM_CENTER*wctrl(i,k))/WHEEL_RADIUS; %Vctrl=0.5 
                wl = (vctrl-DISTANCE_FROM_CENTER*wctrl(i,k))/WHEEL_RADIUS;
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
                robotat_3pi_set_wheel_velocities(pol(i), wl, wr);% Envio de datos de rueda derecha e izquierda
                pose = robotat_get_pose(robot, 3, 'eulxyz')
                xf(k) = pose(1)*100+380/2+10;
                yf(k) = pose(2)*100+480/2+10;
            end
        end
        
        
robotat_3pi_force_stop(pol(1)) % Detiene al pololu 3
robotat_3pi_force_stop(pol(2)) % Detiene al pololu 4
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
%robotat_3pi_force_stop(pol(3)) % Detiene al pololu 5



%% Desconexion del robot
%robotat_3pi_disconnect(5)
robotat_3pi_disconnect(3)
robotat_3pi_disconnect(4) %cambiar al 4
%robotat_3pi_disconnect(5)
robotat_disconnect(robot)

% PROBAR LO DE QUE TAL SIGUEN LOS ROBOTS LA SIMULACION SOLO CORRER ESTO y
% mirar
