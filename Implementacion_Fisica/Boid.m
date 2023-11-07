classdef Boid
    
    properties % Definimos las propiedades del Boid
        position
        velocity
        acceleration
        r
        max_force
        max_speed
        angle
        predator
    end
    
    methods
        function obj = Boid(position_x,  position_y, z_angle,predator) % Definimos los valores iniciales
            obj.acceleration = [0 0];
            obj.angle = z_angle;
            angle = obj.angle;
      
            obj.position = [position_x, position_y];
            obj.r = 0;
            obj.max_speed = 0.3; 
            obj.max_force = 0.1;
            
            obj.predator = predator;
            obj.velocity = [obj.max_speed*cos(angle), obj.max_speed*sin(angle)];
        end
        
        
        function obj = apply_force(obj, sep_force, coh_force,  ali_force, eva_force)
            obj.acceleration = obj.acceleration+sep_force+coh_force+ali_force+eva_force; % Aplicamos la fuerza de las reglas
        end
        
        
        function obj = flock(obj,boids)
            % Obtenemos los valores de las funciones de cada regla
            sep = obj.seperate(boids);
            ali = obj.align(boids);
            coh = obj.cohesion(boids);
            
            eva = obj.evaIA(boids);
            
            % Ajustamos los valores obtenidos
            sep = sep.*0.6;%0.6 
            ali = ali.*0.1; %0.1
            coh = coh.*0.1; %0.1
            eva = eva.*0.1;
            % Aplicamos la fuerza
            obj=obj.apply_force(sep,coh,ali,eva);
        end
        
        function obj = borders(obj, lattice_size)
            vel_cor = 0.5;
            if obj.position(1) < -obj.r
                obj.velocity(1) = obj.velocity(1)+vel_cor*obj.max_speed;	
                obj.acceleration(1) = 0;
            end
            
            if obj.position(2) < -obj.r
                obj.velocity(2) = obj.velocity(2)+vel_cor*obj.max_speed;
                obj.acceleration(2) = 0;
            end
            
            if obj.position(1) > lattice_size(1) + obj.r
                obj.velocity(1) = obj.velocity(1)-vel_cor*obj.max_speed;
                obj.acceleration(1) = 0;
            end
            
            if obj.position(2) > lattice_size(2) + obj.r
                obj.velocity(2) = obj.velocity(2)-vel_cor*obj.max_speed;
                obj.acceleration(1) = 0;
            end
            
        end
        
        function obj = update(obj, boids)
            obj.velocity = obj.velocity + obj.acceleration;
            obj.velocity = obj.velocity./norm(obj.velocity).*obj.max_speed;
            obj.position = obj.position + obj.velocity;
            obj.angle = atan2(obj.velocity(2),obj.velocity(1));
            obj.acceleration = [0 0];
        end
        
        function [steer] = seek(obj, target)
            desired = target - obj.position;
            desired = norm(desired);
            desired = desired*obj.max_speed;
            
            steer = desired-obj.velocity;
            steer = steer./norm(steer).*obj.max_force;
        end
        
        function [steer] = seperate(obj, boids)
            desired_separation = 6;
            steer = [0,0];
            count = 0;
            positions = zeros(2,length(boids));
            for i=1:1:length(boids)
                positions(:,i) = boids(i).position;
            end
            d = pdist([obj.position; positions']);
            d = d(1:length(boids));
            diff = 0;
           %G0 0d
            for i=1:1:length(boids)
                if d(i) > 0 && d(i) <  desired_separation

                    diff = diff - (boids(i).position-obj.position);
                    %difference = difference./norm(difference);
                    %difference = difference./d(i);
                    steer = diff;
                    count = count+1;
                end
                
                if count > 0
                    steer = steer./count;
                end
                
                if norm(steer) > 0
                    diff = steer/8;
                    %diff = steer./norm(steer);
                    angleg = atan2(diff(2),diff(1));
                    eo = angdiff(obj.angle, angleg);
                    eo = rad2deg(eo);
                    steer = [obj.max_speed*(cos(eo)), obj.max_speed*(sin(eo))];
                end
            end
        end
        
        function steer = align(obj, boids)
            neighbor_dist = 25;
            sum = [0 0];
            count = 0;
            steer = [0 0];
            
            positions = zeros(2,length(boids));
            for i=1:1:length(boids)
                positions(:,i) = boids(i).position;
            end
            d = pdist([obj.position; positions']);
            d = d(1:length(boids));
            
            for i=1:1:length(boids)
                if d(i)>6 && d(i) < neighbor_dist %6 porque es el valor del parametro de separaciòn
                    sum=sum+boids(i).velocity;
                    count=count+1;
                end
            end
            
            if count > 0
                sum=sum./count;
                %sum=sum./norm(sum).*obj.max_speed;
                steer=(sum-obj.velocity)/1;
                %steer=steer/8;
                %steer=steer./norm(steer).*obj.max_force;
            end
        end
        
        function steer = cohesion(obj, boids)
            neighbor_dist = 40;
            sum = [0 0];
            count = 0;
            steer = [0 0];
            diff = 0;
            positions = zeros(2,length(boids));
            for i=1:1:length(boids)
                positions(:,i) = boids(i).position;
            end
            d = pdist([obj.position; positions']);
            d = d(1:length(boids));
            
            for i=1:1:length(boids)
                if d(i)>0 && d(i) < neighbor_dist
                    sum=sum+boids(i).position;
                    count=count+1;
                end
            end
            
            if count > 0
                sum=sum./count;
                %diff = sum - (obj.position);
                diff = (sum - (obj.position));
                diff = diff./norm(diff);
                angleg = atan2(diff(2),diff(1));
 
                eo = angdiff(obj.angle, angleg);
                eo = rad2deg(eo);
                %eo = mod(eo, 360);
                %eo = rad2deg(eo)
                steer = [obj.max_speed*(cos(eo)), obj.max_speed*(sin(eo))];
                    
                %sum=sum./norm(sum).*obj.max_speed;
                %steer=sum-obj.position;
                %steer=steer./norm(steer).*obj.max_force;
                
                %steer = obj.seek(sum);
                
                
                
                
            end
        end
        function steer = evaIA(obj, boids)
            steer = [0 0];
            positions = zeros(2,1);
            positionsp = zeros(2,1);
            u = zeros(2,1);
            v = zeros(2,1);
            % Matriz de rotacion para el vector a
            % Definir el ángulo de rotación en radianes (por ejemplo, pi/36 para 5 grados)
            angulo = pi/36;
            % Crear una matriz de rotación 2D
            R = [cos(angulo), -sin(angulo); sin(angulo), cos(angulo)];
            
            positions = obj.position;
      
            u = normr(obj.velocity);
            v = u*25; 
            positionsp = obj.predator.position;
            
            a = positionsp-positions;

            p = (a.*u).*u;
            b = p - a;
            

            if (norm(b)<100) && (norm(p)<norm(v)) % Si b es menor al radio de 100 del depredador y magnitud de p < v

              w = (R * a')';
              wx = w(1);
              %t(:,i) = obj.boids(i).angle 
              if wx<0
                  m = 1;

              end
              if wx>0    
                  m = -1;
                  % Definir el vector original (a)
                  % Realizar la rotación
              end
              
              if wx==0    
                  m = -1;
                  % Definir el vector original (a)
                  % Realizar la rotación
              end
              steer = steer + m*obj.max_speed*(25)/norm(a)
              %steer = steer(1)
              %steer = [obj.max_speed*(cos(eo)), obj.max_speed*(sin(eo))];



            end

            %theta = linspace(0, 2*pi, 100); % Puntos equidistantes alrededor del círculo
            %x = positionsp(1,1) + radiopredator * cos(theta);
            %y = positionsp(2,1) + radiopredator * sin(theta)
            %r = sqrt(x^2+y^2)
            
            end
      
        
        
    end
end