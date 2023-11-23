classdef Boid
    
    properties
        position
        velocity
        acceleration
        r
        max_force
        max_speed
        angle
        wvelocity
        wacceleration
    end
    
    methods
%        function obj = Boid(position_x,  position_y, z_angle)
        function obj = Boid(position_x,  position_y, z_angle)
            obj.acceleration = [0 0];
            obj.wacceleration = 0;
            %obj.angle = (2*pi).*rand;
            obj.angle = z_angle;
            angle = obj.angle;
            obj.wvelocity = [0]; 
            %obj.velocity = [cos(angle), sin(angle)];
            
            obj.position = [position_x, position_y];
            obj.r = 0;
            obj.max_speed = 0.3; %0.06 y con un pause de 0.265 funciona bien 0.3 tambien
            obj.max_force = 0.1;
            
            obj.velocity = [obj.max_speed*cos(angle), obj.max_speed*sin(angle)];
        end
        
        
        function obj = apply_force(obj, sep_force, coh_force,  ali_force, ten_force)
            obj.acceleration = obj.acceleration+sep_force+coh_force+ali_force+ten_force;
        end
        
        
        function obj = flock(obj,boids)
            sep = obj.seperate(boids);
            ali = obj.align(boids);
            coh = obj.cohesion(boids);
            
            tend = obj.ten(boids);
            
            sep = sep.*0.6;%0.6
            ali = ali.*0.1; %0.1
            coh = coh.*0.1; %0.1
            tend = tend.*0.0;
            obj=obj.apply_force(sep,coh,ali,tend);
        end
        
        function obj = borders(obj, lattice_size)
            vel_cor = 0.5;
            if obj.position(1) < -obj.r
                %obj.position(1)=lattice_size(1)+obj.r;
                %obj.position(1)=-obj.r;
                obj.velocity(1) = obj.velocity(1)+vel_cor*obj.max_speed;
				
				
                obj.acceleration(1) = 0;
                %obj.acceleration(1)=-obj.acceleration(1);
            end
            
            if obj.position(2) < -obj.r
                %obj.position(2)=lattice_size(2)+obj.r;
                %obj.position(2)=-obj.r;
                obj.velocity(2) = obj.velocity(2)+vel_cor*obj.max_speed;
                obj.acceleration(2) = 0;
                %obj.acceleration(2)=-obj.acceleration(2);
            end
            
            if obj.position(1) > lattice_size(1) + obj.r
                %obj.position(1)=-obj.r;
                %obj.position(1)=lattice_size(1)+obj.r;
                obj.velocity(1) = obj.velocity(1)-vel_cor*obj.max_speed;
                obj.acceleration(1) = 0;
                %obj.acceleration(1)=-obj.acceleration(1);
            end
            
            if obj.position(2) > lattice_size(2) + obj.r
                %obj.position(2)=lattice_size(2)+obj.r;
                %obj.position(2)=-obj.r;
                obj.velocity(2) = obj.velocity(2)-vel_cor*obj.max_speed;
                obj.acceleration(1) = 0;
                %obj.acceleration(2)=-obj.acceleration(2);
            end
            
        end
        
        function obj = update(obj, boids)
            %angles = zeros(1,length(boids));
            %for i=1:1:length(boids)
            %    prevangles(:,i) = boids(i).angle
            %end
            obj.velocity = obj.velocity + obj.acceleration;
            %obj.wvelocity = obj.wvelocity + obj.wacceleration; %Meter esto en las funciones
            obj.velocity = obj.velocity./norm(obj.velocity).*obj.max_speed;
            obj.position = obj.position + obj.velocity;
            obj.angle = atan2(obj.velocity(2),obj.velocity(1));
            %obj.angle = obj.angle + obj.wvelocity; %METER ESTO EN LAS FUNCIONES
            obj.acceleration = [0 0];
            
            %for i=1:1:length(boids)
            %    newangles(:,i) = boids(i).angle
            %end
            
            %totangles = newangles-angles
            %p = obj.angle()
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
                    %steer = steer;
                    %posp = obj.position/norm(obj.position);
                    %diff = steer - (obj.position/norm(obj.position));
                    diff = steer./norm(steer);
                    angleg = atan2(diff(2),diff(1));
                    %obj.angle = obj.angle; Usado para comparar 
                    eo = angdiff(obj.angle, angleg);
                    eo = rad2deg(eo);
                    %eo = mod(eo, 360);
                    %eo = rad2deg(eo)
                    steer = [obj.max_speed*(cos(eo)), obj.max_speed*(sin(eo))];
                    
%                     steer = steer./norm(steer).*obj.max_speed;
%                     steer = steer - obj.velocity;
%                     steer = steer./norm(steer).*obj.max_force;
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
                steer=sum-obj.velocity;
                %steer=steer/8;
                %steer=steer./norm(steer).*obj.max_force;
            end
        end
        
        function steer = cohesion(obj, boids)
            neighbor_dist = 40;
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
                if d(i)>0 && d(i) < neighbor_dist
                    sum=sum+boids(i).position;
                    count=count+1;
                end
            end
            
            if count > 0
                sum=sum./count;
                %diff = sum - (obj.position);
                diff = sum - (obj.position);
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
        
        function steer = ten(obj, boids)
            steer = [0 0];
            d = pdist([obj.position; [380/2,380/2]]);
            d = d(1:1);
            positions = zeros(2,length(boids));
 
            diff = (obj.position)-[380/2, 380/2];
            angleg = atan2(diff(2),diff(1));
            eo = angdiff(obj.angle, angleg);
            eo = rad2deg(eo);
            steer = [obj.max_speed*(cos(eo)), obj.max_speed*(sin(eo))];
        end
        
        
    end
end
%%
