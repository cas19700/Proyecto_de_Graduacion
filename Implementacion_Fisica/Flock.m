
classdef Flock
    %FLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        boids
        predator
        lattice_size
        rad_sp
        step_counter=1;
        ftimes
        val_rad
        pololu
        wlf
        wrf
        prevang
        ant_sp
        xval 
        yval 
        arr_sp
        robotat
        simu_time
        xvals
        yvals
    end
    
    methods
        
        function obj = Flock(boids,lattice_size,pololu,robotat,predator)
            obj.boids=boids;
            obj.predator=predator;
            obj.lattice_size=lattice_size;
            obj.rad_sp=zeros(1,length(boids));
            obj.ftimes = 0;
            obj.val_rad = 0;
            obj.pololu = pololu;
            obj.wlf = 0;
            obj.wrf = 0;
            obj.prevang = 0;
            obj.ant_sp = [0.001, 0.001];
            obj.robotat = robotat;
            %obj.xval = obj.boids(1).position(1); % Generamos el array
            %obj.yval = obj.boids(1).position(2); % Generamos el array
            
            %obj.xvals = [obj.boids(1).position(1)]; % Generamos el array
            %obj.yvals = [obj.boids(1).position(2)]; % Generamos el array
            obj.simu_time = 150;
            obj.xvals = zeros(1,obj.simu_time)
            obj.yvals = zeros(1,obj.simu_time)
            obj.xvals(1,1) = obj.boids(1).position(1); % Generamos el array PROBAR QUITAR ESTO
            obj.yvals(1,1) = obj.boids(1).position(2); % Generamos el array
            obj.xvals(2,1) = obj.boids(2).position(1); % Generamos el array PROBAR QUITAR ESTO
            obj.yvals(2,1) = obj.boids(2).position(2); % Generamos el array
            
            
            obj.arr_sp = zeros(length(boids),length(boids))
            %cor = robotat_get_pose(obj.robotat, 8, 'eulxyz');
            
         
            %obj.xval(1) = cor(1)*100+380/2+10; 
            %obj.yval(1) = cor(2)*100+480/2+10; %CREO QUE ESTE TIENE QUE SER 480
            %vx = obj.xval(1) %Pruebas
            %vy = obj.yval(1)
            %fprintf('VALORES\n', vx)
            %fprintf('VALORES\n', vy)
        end
        
        function obj = run(obj, plane)
            while (obj.step_counter<=obj.simu_time)
                
                for i=1:length(obj.boids)
                %ftot(:,1) = ftime
                    prevang(:,i) = obj.boids(i).angle;
                    obj.prevang = prevang;
                end
                %obj = borders(obj);
                obj = flock(obj);
                obj = update_boids(obj);
                obj = borders(obj);
                
                %obj = flock(obj);
                %obj = update_boids(obj);
                %obj = update_boids(obj);
               
                
                [obj,plane] = render(obj,plane);
                
            end
        end
        
        function obj = update_boids(obj)
            ftime = 0;
            
            for i=1:length(obj.boids)
                %ftot(:,1) = ftime
                %prevang(:,i) = obj.boids(i).angle
                obj.boids(i)=obj.boids(i).update(obj.boids);
                %toc;
                %newang(:,i) = obj.boids(i).angle
                %radialsp = (newang - obj.prevang)
                %obj.rad_sp=radialsp;
                %ftime=toc
                %fprintf('Tiempo transcurrido: %.4f segundos\n', ftime);
                %ftot(:,2) = ftime
                %res = ftot(:,2)-ftot(:,1)
            end
        end
        
        function obj = flock(obj)
            for i=1:length(obj.boids)
                obj.boids(i)=obj.boids(i).flock(obj.boids);
            end
        end
        
        
        function [obj,plane] = render(obj,plane)
            obj.step_counter=obj.step_counter+1;
            fprintf('Rendering %s \n',num2str(obj.step_counter))
            
            
                        v = obj.xvals;
                        fprintf('VALORES\n', v);
            %tEnd = toc(tStart);
            
            for i=1:length(obj.boids)
                delete(plane.boids_figure_handles(i));
                theta = atan2(obj.boids(i).velocity(2),obj.boids(i).velocity(1));
                x = [obj.boids(i).position(1)-2.5 obj.boids(i).position(1)+2.5 obj.boids(i).position(1)-2.5 obj.boids(i).position(1)-2.5];
                y = [obj.boids(i).position(2)-1.5 obj.boids(i).position(2) obj.boids(i).position(2)+1.5 obj.boids(i).position(2)+1.5];
                plane.boids_figure_handles(i) =  patch(x,y,'k');
                rotate(plane.boids_figure_handles(i), [0 0 1], rad2deg(theta), [obj.boids(i).position(1) obj.boids(i).position(2) 0]);
                %cor = robotat_get_pose(obj.robotat, 8, 'eulxyz');
                
                %obj.xval(obj.step_counter) = obj.boids(1).position(1);
                %obj.yval(obj.step_counter) = obj.boids(1).position(2);
                if obj.step_counter == 3
                    xp = obj.predator(1).position(1);
                    yp = obj.predator(1).position(2);
                    plane.predator_figure_handles(1) = viscircles([xp,yp], 10, 'EdgeColor', 'k', 'LineWidth', 2);
                    %plot(xp, yp, 'o', 'MarkerSize', 100, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k');
                end   
                
                obj.xvals(i,obj.step_counter) = obj.boids(i).position(1);
                obj.yvals(i,obj.step_counter) = obj.boids(i).position(2);
                
                %obj.xval(obj.step_counter) = cor(1)*100+380/2+10;
                %obj.yval(obj.step_counter) = cor(2)*100+480/2+10;
                 
                
                grid on;
            end
            drawnow;
            
            if obj.step_counter >= obj.simu_time
                hold on;
            
                %plot (obj.xval(1,:), obj.yval(1,:));		
                
                plot (obj.xvals(1,:), obj.yvals(1,:));
                
                p = obj.xvals(148)
                
                %obj.yvals = obj.yvals
                %robotat_3pi_force_stop(obj.pololu(1)) % Detiene al pololu 3
                %robotat_3pi_force_stop(obj.pololu(2)) % Detiene al pololu 4
            end
            
            
             for i=1:length(obj.boids)
                %ftot(:,1) = ftime
                    newang(:,i) = obj.boids(i).angle;
             end
             newang=newang; %Quitar coma para visualizar
             %prevang=obj.prevang
             radialsp = (newang - obj.prevang); %Quitar coma visualizar
             obj.rad_sp=radialsp;   %Quitar coma visualizar
             %if obj.rad_sp > 50
             %   obj.rad_sp = obj.rad_sp
             %end
            
            ftime = obj.ftimes;
            ftot(:,1) = ftime;
            %toc;
            ftime=toc;
            fprintf('Tiempo transcurrido: %.4f segundos\n', ftime)
            ftot(:,2) = ftime;
            res = ftot(:,2)-ftot(:,1);
            obj.ftimes = ftot(:,2);
            obj.val_rad = res;
            
            %ENVIO A ROBOTS
            %t = flock.ftimes;
            %wd = flock.val_rad;
            obj.rad_sp=obj.rad_sp;
            
            wctrl = obj.rad_sp/obj.ftimes
            %zeta = length(obj.pololu)
            obj.arr_sp(:,obj.step_counter) = wctrl'
            
            
            
           

             % FINAL DEL FOR PARA ENVIO DE DATOS A POLOLUS
            %pause(0.05)
            %toc
        end
        
        function obj = borders(obj)
            for i=1:length(obj.boids)
                obj.boids(i) = obj.boids(i).borders(obj.lattice_size);
            end
        end
        
        %          
    end
    
end

