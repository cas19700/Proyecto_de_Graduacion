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
        xval
        yval
    end
    
    methods
        
        function obj = Flock(boids,lattice_size,predator)
            obj.boids=boids;
            obj.predator=predator;
            obj.lattice_size=lattice_size;
            obj.rad_sp=zeros(1,length(boids));
            obj.ftimes = 0;
            obj.val_rad = 0;
            
            obj.xval = obj.boids(1).position(1); % Generamos el array
            obj.yval = obj.boids(1).position(2); % Generamos el array
            
             
        end
        
        function run(obj, plane)
            while true
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
                prevang(:,i) = obj.boids(i).angle;
                obj.boids(i)=obj.boids(i).update(obj.boids);
                %toc;
                newang(:,i) = obj.boids(i).angle;
                radialsp = newang - prevang;
                obj.rad_sp=radialsp;
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
            %tEnd = toc(tStart);
            
            for i=1:length(obj.boids)
                delete(plane.boids_figure_handles(i));
                theta = atan2(obj.boids(i).velocity(2),obj.boids(i).velocity(1));
                x = [obj.boids(i).position(1)-2.5 obj.boids(i).position(1)+2.5 obj.boids(i).position(1)-2.5 obj.boids(i).position(1)-2.5];
                y = [obj.boids(i).position(2)-1.5 obj.boids(i).position(2) obj.boids(i).position(2)+1.5 obj.boids(i).position(2)+1.5];
                plane.boids_figure_handles(i) =  patch(x,y,'k');
                rotate(plane.boids_figure_handles(i), [0 0 1], rad2deg(theta), [obj.boids(i).position(1) obj.boids(i).position(2) 0]);
                obj.xval(obj.step_counter) = obj.boids(1).position(1);
                obj.yval(obj.step_counter) = obj.boids(1).position(2);
                if obj.step_counter == 5
                    xp = obj.predator(1).position(1);
                    yp = obj.predator(1).position(2);
                    plane.predator_figure_handles(1) = viscircles([xp,yp], 50, 'EdgeColor', 'k', 'LineWidth', 2);
                    %plot(xp, yp, 'o', 'MarkerSize', 100, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k');
                end
            end
            
            if obj.step_counter > 3
                hold on;
                grid on;
                %plot (obj.xval, obj.yval);
            end
            
             v = obj.xval;%COMENTAR
                        fprintf('VALORES\n', v);
            drawnow;
            ftime = obj.ftimes;
            ftot(:,1) = ftime;
            %toc;
            ftime=toc;
            fprintf('Tiempo transcurrido: %.4f segundos\n', ftime);
            ftot(:,2) = ftime;
            res = ftot(:,2)-ftot(:,1);
            obj.ftimes = ftot(:,2);
            obj.val_rad = res;
            wctrl = obj.val_rad/obj.ftimes;
            %pause(1)
            %toc
        end
        
        function obj = borders(obj)
            for i=1:length(obj.boids)
                obj.boids(i) = obj.boids(i).borders(obj.lattice_size);
            end
        end
        
        %           void render() {
        %     // Draw a triangle rotated in the direction of velocity
        %     float theta = velocity.heading2D() + radians(90);
        %     // heading2D() above is now heading() but leaving old syntax until Processing.js catches up
        %
        %     fill(200, 100);
        %     stroke(255);
        %     pushMatrix();
        %     translate(position.x, position.y);
        %     rotate(theta);
        %     beginShape(TRIANGLES);
        %     vertex(0, -r*2);
        %     vertex(-r, r*2);
        %     vertex(r, r*2);
        %     endShape();
        %     popMatrix();
        %   }
        
    end
    
end

