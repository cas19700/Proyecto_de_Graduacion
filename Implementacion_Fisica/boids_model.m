%% Simulación de Boids de Reynolds
% Compu path: C:\Users\Brayan\OneDrive - MABREX CENTROAMERICA S.A\Documentos\Año 2023\Segundo semestre\Tesis\PruebasFisicas
%% Variables Implementación Física 
clear all;
%tStart = tic;

robot = robotat_connect();
pol(1) = robotat_3pi_connect(2)
pol(2) = robotat_3pi_connect(3)  
pol(3) = robotat_3pi_connect(4)
pol(4) = robotat_3pi_connect(5)
pol(5) = robotat_3pi_connect(6)
%pol(6) = robotat_3pi_connect(7)
%pol(7) = robotat_3pi_connect(8)
%pol(8) = robotat_3pi_connect(9)

MAX_WHEEL_VELOCITY = 850; % Velocidad máxima ruedas (en rpm) 
MAX_WHEEL_VELOCITY = 850*(2*pi/60); % Velocidad máxima ruedas (en rad/s)
WHEEL_RADIUS = (32/2)/1000; % Radio de las ruedas (en m)
DISTANCE_FROM_CENTER = (94/2)/1000; % Distancia a ruedas (en cm)
MAX_SPEED = WHEEL_RADIUS * MAX_WHEEL_VELOCITY;% Velocidad lineal máxima (en cm/s)

%% Pruebas Robots Pololu 3pi+
% Usado para comprobar que los Pololu funcionen adecuadamente
robotat_3pi_set_wheel_velocities(pol(1),100,-100)
robotat_3pi_force_stop(pol(1))


robotat_3pi_set_wheel_velocities(pol(2),100,-100)
robotat_3pi_force_stop(pol(2))


robotat_3pi_set_wheel_velocities(pol(3),100,-100)
robotat_3pi_force_stop(pol(3))


%% Obtener posiciones de los Pololu

% Pololu #2 CAMBIAR A 3, ESTA EN 6 CAMBIAR ESTE SIEMPRE EN EL FLOCK TAMBIEN
pos1 = robotat_get_pose(robot, 2, 'eulxyz') % Obtener la posicion del pololu y angulos
bearing1 = pos1(6)+143.3192;                 % Ajuste para el angulo de bearing
bearing1 = deg2rad(bearing1);                % Cambio a radianes

% Pololu #3   CAMBIAR A 4 ESTA EN 8(HOY ESTABAN USANDOLO)
pos2 = robotat_get_pose(robot, 3, 'eulxyz') % Obtener la posicion del pololu y angulos
bearing2 = pos2(6)-176.4374;                 % Ajuste para el angulo de bearing
bearing2 = deg2rad(bearing2);                % Cambio a radianes

% Pololu #4
pos3 = robotat_get_pose(robot, 4, 'eulxyz') % Obtener la posicion del pololu y angulos
bearing3 = pos3(6)-131.4114;                 % Ajuste para el angulo de bearing
bearing3 = deg2rad(bearing3);                % Cambio a radianes


% Pololu #5
pos4 = robotat_get_pose(robot, 5, 'eulxyz') % Obtener la posicion del pololu y angulos
bearing4 = pos4(6)-94.1125;                 % Ajuste para el angulo de bearing
bearing4 = deg2rad(bearing4);                % Cambio a radianes

% Pololu #6
pos5 = robotat_get_pose(robot, 6, 'eulxyz') % Obtener la posicion del pololu y angulos
bearing5 = pos5(6)-130.5976;                 % Ajuste para el angulo de bearing
bearing5 = deg2rad(bearing5);                % Cambio a radianes
%% Pololu #7
pos6 = robotat_get_pose(robot, 7, 'eulxyz') % Obtener la posicion del pololu y angulos
bearing6 = pos6(6)-99.5497;                 % Ajuste para el angulo de bearing
bearing6 = deg2rad(bearing6);                % Cambio a radianes
%% Pololu #8
pos7 = robotat_get_pose(robot, 8, 'eulxyz') % Obtener la posicion del pololu y angulos
bearing7 = pos7(6)+168.1945;                 % Ajuste para el angulo de bearing
bearing7 = deg2rad(bearing7);                % Cambio a radianes
%% Pololu #9
pos8 = robotat_get_pose(robot, 9, 'eulxyz') % Obtener la posicion del pololu y angulos
bearing8 = pos8(6)+103.7972;                 % Ajuste para el angulo de bearing
bearing8 = deg2rad(bearing8);                % Cambio a radianes
%% Simulación
boids_count=5; % Numero de pololus a utilizar
predator_count=1;
predator=Predator.empty;
boids=Boid.empty; % Espacios
predator=Predator(230,250)

%% Ajuste de plano
% Pololu #2
boids(1) = Boid(pos1(1)*100+340/2,pos1(2)*100+440/2,bearing1,predator) % Envios de posiciones, angulo de bearing y desfases (340/2)
                                                                 % El +10 tambien es para un desfase y hacer 
                                                                 % mas pequeño el mapa                                                          
% Pololu #3
boids(2) = Boid(pos2(1)*100+340/2,pos2(2)*100+440/2,bearing2,predator) % Envios de posiciones, angulo de bearing y desfases (340/2)
  %boids(2)                                                               % El +10 tambien es para un desfase y hacer 
                                                                 % mas pequeño el mapa
% Pololu #4                                                                 
boids(3) = Boid(pos3(1)*100+340/2,pos3(2)*100+440/2,bearing3,predator) % Envios de posiciones, angulo de bearing y desfases (340/2)
                                                                 % El +10 tambien es para un desfase y hacer 
                                                                 % mas pequeño el mapa                                                                
% Pololu #5                                                                 
boids(4) = Boid(pos4(1)*100+340/2,pos4(2)*100+440/2,bearing4,predator) % Envios de posiciones, angulo de bearing y desfases (340/2)
%boids(4)                                                                 % El +10 tambien es para un desfase y hacer 
                                                                 % mas pequeño el mapa                                                                
% Pololu #6                                                                 
boids(5) = Boid(pos5(1)*100+340/2,pos5(2)*100+440/2,bearing5,predator) % Envios de posiciones, angulo de bearing y desfases (340/2)
%boids(5)                                                                 % El +10 tambien es para un desfase y hacer 
                                                                 % mas pequeño el mapa                                                                
%% Pololu #7                                                                 
boids(6) = Boid(pos6(1)*100+340/2,pos6(2)*100+440/2,bearing6,predator) % Envios de posiciones, angulo de bearing y desfases (340/2)
%boids(5)                                                                 % El +10 tambien es para un desfase y hacer 
                                                                 % mas pequeño el mapa                                                                
    
% Pololu #8                                                                 
boids(7) = Boid(pos7(1)*100+340/2,pos7(2)*100+440/2,bearing7,predator) % Envios de posiciones, angulo de bearing y desfases (340/2)
%boids(5)                                                                 % El +10 tambien es para un desfase y hacer 
                                                                 % mas pequeño el mapa                                                                
% Pololu #9                                                             
boids(8) = Boid(pos8(1)*100+340/2,pos8(2)*100+440/2,bearing8,predator) % Envios de posiciones, angulo de bearing y desfases (340/2)
%boids(5)                                                                 % El +10 tambien es para un desfase y hacer 
                                                                 % mas pequeño el mapa                                                                
%% Ejecutar simulación                                                                
tic
%pol = pol
flock=Flock(boids,[340 440], pol,robot,predator);% Se cargan las dimensiones de los bordes del programa (x=340, y=440)
trad = flock.ftimes % Esto creo que ya no lo use
fprintf('Tiempos:'); % Esto tampoco
f = figure;
plane = Plane(f,[340 440],boids,robot,predator) % Se visualiza la simulación
flocksim = flock.run(plane);   % Se empiezan a correr los renderizados
%% Controlador e inicialización de registro de datos
% PID orientación
kpO = 2*5;
kiO = 0.0001; 
kdO = 0;
EO = 0;
eO_1 = 0;

% Acercamiento exponencial
v0 = 0.25;
alpha = 0.5;
nval = [1,1,1,1,1];
% Arreglos para generar los espacios de memoria
cor = zeros(1,2);
xf = zeros(1,2);
yf = zeros(1,2);
xf(1,1) = pos1(1)*100+340/2; % Ajustamos el eje x para hacer que concuerde con la plataforma
yf(1,1) = pos1(2)*100+440/2;

xf(2,1) = pos2(1)*100+340/2; % Ajustamos el eje x para hacer que concuerde con la plataforma
yf(2,1) = pos2(2)*100+440/2;

xf(3,1) = pos3(1)*100+340/2; % Ajustamos el eje x para hacer que concuerde con la plataforma
yf(3,1) = pos3(2)*100+440/2;

xf(4,1) = pos4(1)*100+340/2; % Ajustamos el eje x para hacer que concuerde con la plataforma
yf(4,1) = pos4(2)*100+440/2;

xf(5,1) = pos5(1)*100+340/2; % Ajustamos el eje x para hacer que concuerde con la plataforma
yf(5,1) = pos5(2)*100+440/2;
%%
xf(6,1) = pos6(1)*100+340/2; % Ajustamos el eje x para hacer que concuerde con la plataforma
yf(6,1) = pos6(2)*100+440/2;

xf(7,1) = pos7(1)*100+340/2; % Ajustamos el eje x para hacer que concuerde con la plataforma
yf(7,1) = pos7(2)*100+440/2;

xf(8,1) = pos8(1)*100+340/2; % Ajustamos el eje x para hacer que concuerde con la plataforma
yf(8,1) = pos8(2)*100+440/2;
%% Puntos a alcanzar 
xgoal = flocksim.xvals; 
ygoal = flocksim.yvals;
q = 0;
%% Suavizados (Interpolación)
% Suavizado del primer Pololu
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
grid on
%% Suavizado del segundo Pololu

cpts2 = [xgoal(2,:);ygoal(2,:)]
tpts2 = [0 25];
tvect2 = 0:5:25
%tpts = [0 150];
%tvect = 0:1:150

[q2, qd2, qdd2, pp2] = bsplinepolytraj(cpts2,tpts2,tvect2); % Se obtienen valores mas suavizados
figure
plot(cpts2(1,:),cpts2(2,:),'xb-') % Se grafican los valores originales
hold all
plot(q2(1,:), q2(2,:)) % Se grafican los nuevos valores
xlabel('X')
ylabel('Y')
hold off
grid on

%% Suavizado del tercer Pololu

cpts3 = [xgoal(3,:);ygoal(3,:)]
tpts3 = [0 25];
tvect3 = 0:5:25
%tpts = [0 150];
%tvect = 0:1:150

[q3, qd3, qdd3, pp3] = bsplinepolytraj(cpts3,tpts3,tvect3); % Se obtienen valores mas suavizados
figure
plot(cpts3(1,:),cpts3(2,:),'xb-') % Se grafican los valores originales
hold all
plot(q3(1,:), q3(2,:)) % Se grafican los nuevos valores
xlabel('X')
ylabel('Y')
hold off
grid on

%% Suavizado del cuarto Pololu

cpts4 = [xgoal(4,:);ygoal(4,:)]
tpts4 = [0 25];
tvect4 = 0:5:25
%tpts = [0 150];
%tvect = 0:1:150

[q4, qd4, qdd4, pp4] = bsplinepolytraj(cpts4,tpts4,tvect4); % Se obtienen valores mas suavizados
figure
plot(cpts4(1,:),cpts4(2,:),'xb-') % Se grafican los valores originales
hold all
plot(q4(1,:), q4(2,:)) % Se grafican los nuevos valores
xlabel('X')
ylabel('Y')
hold off
grid on

%% Suavizado del quinto Pololu

cpts5 = [xgoal(5,:);ygoal(5,:)]
tpts5 = [0 25];
tvect5 = 0:5:25
%tpts = [0 150];
%tvect = 0:1:150

[q5, qd5, qdd5, pp5] = bsplinepolytraj(cpts5,tpts5,tvect5); % Se obtienen valores mas suavizados
figure
plot(cpts5(1,:),cpts5(2,:),'xb-') % Se grafican los valores originales
hold all
plot(q5(1,:), q5(2,:)) % Se grafican los nuevos valores
xlabel('X')
ylabel('Y')
hold off
grid on

%% Suavizado del sexto Pololu

cpts6 = [xgoal(6,:);ygoal(6,:)]
tpts6 = [0 25];
tvect6 = 0:5:25
%tpts = [0 150];
%tvect = 0:1:150

[q6, qd6, qdd6, pp6] = bsplinepolytraj(cpts6,tpts6,tvect6); % Se obtienen valores mas suavizados
figure
plot(cpts6(1,:),cpts6(2,:),'xb-') % Se grafican los valores originales
hold all
plot(q6(1,:), q6(2,:)) % Se grafican los nuevos valores
xlabel('X')
ylabel('Y')
hold off
grid on

%% Suavizado del septimo Pololu

cpts7 = [xgoal(7,:);ygoal(7,:)]
tpts7 = [0 25];
tvect7 = 0:5:25
%tpts = [0 150];
%tvect = 0:1:150

[q7, qd7, qdd7, pp7] = bsplinepolytraj(cpts7,tpts7,tvect7); % Se obtienen valores mas suavizados
figure
plot(cpts7(1,:),cpts7(2,:),'xb-') % Se grafican los valores originales
hold all
plot(q7(1,:), q7(2,:)) % Se grafican los nuevos valores
xlabel('X')
ylabel('Y')
hold off
grid on
%% Suavizado del octavo Pololu

cpts8 = [xgoal(8,:);ygoal(8,:)]
tpts8 = [0 25];
tvect8 = 0:5:25
%tpts = [0 150];
%tvect = 0:1:150

[q8, qd8, qdd8, pp8] = bsplinepolytraj(cpts8,tpts8,tvect8); % Se obtienen valores mas suavizados
figure
plot(cpts8(1,:),cpts8(2,:),'xb-') % Se grafican los valores originales
hold all
plot(q8(1,:), q8(2,:)) % Se grafican los nuevos valores
xlabel('X')
ylabel('Y')
hold off
grid on
%% Envío y recepción de datos
q = [q;q2;q3;q4;q5];
%q = [q];
reff=0
bear_m = [0,143.3192,-176.4374,-132.0875,-94.1125,-130.5976,-99.5497,-168.2067,103.7972] % angulos de bearing obtenidos anteriormente
tam1 = [length(q),length(q2),length(q3),length(q4),length(q5)];
%tam1 = [length(q)];
k=1;
detener = zeros(1,length(boids_count));
        while(1)%for k=1:tam1    
            for i=1:5 %boids_count
                roba = robotat_get_pose(robot, 1+i, 'eulxyz'); % i + 1, Obtener la posicion actual
                bearing = roba(6)+bear_m(1+i); % Ajustar el angulo de bearing
                bearing = deg2rad(bearing); % Grados a radianes
                x = roba(1)*100+340/2; % Ajustamos el eje x para hacer que concuerde con la plataforma
                y = roba(2)*100+440/2; % Ajustamos el eje y para hacer que concuerde con la plataforma
                theta = bearing;   % theta igual al angulo de bearing
                %ref = [q(1,nval); q(2,nval)]; % Referencia igual a los valores obtenidos anteriormente
                ref = [q(i*2-1,nval(i)); q(i*2,nval(i))]; % Referencia igual a los valores obtenidos anteriormente
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
                xv = abs(ref(1))-abs(x);
                yv = abs(ref(2))-abs(y);  
                refff(1,k) = abs(xv);
                refff(2,k) = abs(yv);
                if (abs(xv)<=3) % Comprobamos que este a 10 cm por lo menos para el cambio de punto
                      
                   reff(1,k) = xv;
                   %reff(2,k) = yv 
                   reff(3,k) = abs(xv);
                   %reff(4,k) = abs(yv)
                   %nval = nval+1 % Aumentar el contador
                   
                   if (abs(yv)<=3)
                       reff(2,k) = yv; 
                       reff(4,k) = abs(yv);
                       nval(i) = nval(i)+1 % Aumentar el contador
                       
                       pose = robotat_get_pose(robot, 1+i, 'eulxyz');
                       xf(i,nval(i)) = pose(1)*100+340/2; % Guardar valores de x
                       yf(i,nval(i)) = pose(2)*100+440/2; % Guardar valores de y
                       
                   end
                       
                       if (nval(i) > tam1(i))
                           nval(i) = tam1(i);
                           robotat_3pi_force_stop(pol(i)) % Si llega al ultimo valor detenerse
                           %detener(1,i) = 1;
                           detener = 1;
                           continue;
                       end
                end
                
                  
                wr = (u(1)+DISTANCE_FROM_CENTER*u(2))/WHEEL_RADIUS; % Velocidad de la llanta derecha
                wl = (u(1)-DISTANCE_FROM_CENTER*u(2))/WHEEL_RADIUS; % Velocidad de la llanta izquierda

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
robotat_3pi_force_stop(pol(2)) % Detiene al pololu 4
robotat_3pi_force_stop(pol(3)) % Detiene al pololu 4
robotat_3pi_force_stop(pol(4)) % Detiene al pololu 4
robotat_3pi_force_stop(pol(5)) % Detiene al pololu 4
robotat_3pi_force_stop(pol(6)) % Detiene al pololu 4
robotat_3pi_force_stop(pol(7)) % Detiene al pololu 4
robotat_3pi_force_stop(pol(8)) % Detiene al pololu 4
% bsplinepolytraj
%% Gráficas
figure(18)
q1xf = q(1,1:6)
q1yf = q(2,1:6)
plot(q1xf, q1yf) % Valores de la simulacion interpolados
hold on;
xf1 = xf(1,2:7)
yf1 = yf(1,2:7)
plot(xf1,yf1) % Valores obtenidos en la implementación fisica
% Definir las trayectorias
trayectoria1 = [q1xf(1,:);q1yf(1,:)]';
trayectoria2 = [xf1;yf1]';

% Calcular la diferencia entre las trayectorias
diferencia = trayectoria1 - trayectoria2;
sqr_err = diferencia.^2
mean_squared_error = mean(sqr_err(:)); % ESTE es EC
root_mean_squared_error = sqrt(mean_squared_error); % Este es ECM

%Porcentaje
a = sqrt(sum(trayectoria1.^2,2))
b = sqrt(sum(trayectoria2.^2,2))
c = abs(((a-b)/a))*100
% Calcular la distancia euclidiana al cuadrado entre los puntos
distancias_cuadradas = sum(diferencia.^2, 2);

% Calcular el Error Cuadrático Medio para cada punto
ecm_por_punto = sqrt(distancias_cuadradas); % SACAR ESTOS PUNTOS PARA EL DOCUMENTO

% Mostrar el resultado
disp('Error Cuadrático Medio para cada punto es:');
disp(ecm_por_punto);


% RECORDAR ACTIVAR LAS COR EN FLOCK
err = immse([q1xf(1,:);q1yf(1,:)], [xf1;yf1]); % Error cuadratico medio
fprintf('\n El error cuadratico medio es %0.4f\n', err);

%% Segundo pololu
figure(19)
q2xf = q2(1,1:6)
q2yf = q2(2,1:6)
plot(q2xf, q2yf) % Valores de la simulacion interpolados
%axis([min(flocksim.xvals), max(flocksim.xvals), min(flocksim.yvals), max(flocksim.yvals)]);
hold on;
xf2 = xf(2,2:7)
yf2 = yf(2,2:7)
plot(xf2,yf2) % Valores obtenidos en la implementación fisica
% RECORDAR ACTIVAR LAS COR EN FLOCK
%% Tercer Pololu
figure(20)
q3xf = q3(1,1:6)
q3yf = q3(2,1:6)
plot(q3xf, q3yf) % Valores de la simulacion interpolados
%axis([min(flocksim.xvals), max(flocksim.xvals), min(flocksim.yvals), max(flocksim.yvals)]);
hold on;
xf3 = xf(3,2:7)
yf3 = yf(3,2:7)
plot(xf3,yf3)

%Error cuadratico medio
%% Cuarto Pololu
figure(20)
q4xf = q4(1,1:6)
q4yf = q4(2,1:6)
plot(q4xf, q4yf) % Valores de la simulacion interpolados
%axis([min(flocksim.xvals), max(flocksim.xvals), min(flocksim.yvals), max(flocksim.yvals)]);
hold on;
xf4 = xf(4,2:7)
yf4 = yf(4,2:7)
plot(xf4,yf4)
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
robotat_3pi_disconnect(4)
robotat_3pi_disconnect(5)
robotat_3pi_disconnect(6)
robotat_3pi_disconnect(7)
robotat_3pi_disconnect(8)
robotat_3pi_disconnect(9) %cambiar al 4
%robotat_3pi_disconnect(5)
robotat_disconnect(robot)

% PROBAR LO DE QUE TAL SIGUEN LOS ROBOTS LA SIMULACION SOLO CORRER ESTO y
% mirar
