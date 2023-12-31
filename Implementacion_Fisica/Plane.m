classdef Plane
    %PLANE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        boids_figure_handles
        predator_figure_handles
        plane_handle
        lattice_size
        
        xval 
        yval 
        robotat
    end
    
    methods
        function obj = Plane(plane_handle, lattice_size, boids, robotat, predator)
            obj.plane_handle = plane_handle;
            %             set(p,'xlim',[0 lattice_size(1)]);
            %             set(p,'ylim',[0 lattice_size(2)]);
            
            obj.lattice_size = lattice_size;
            plot(0,0);
            xlim([0 lattice_size(1)]);
            ylim([0 lattice_size(2)]);
            
            %obj.xval = zeros(1, 500); % Generamos el array
            %obj.yval = zeros(1, 500); % Generamos el array
            obj.robotat = robotat;
            for i=1:length(boids)
                
                x = [boids(i).position(1)-2.5 boids(i).position(1)+2.5 boids(i).position(1)-2.5 boids(i).position(1)-2.5];
                y = [boids(i).position(2)-1.5 boids(i).position(2) boids(i).position(2)+1.5 boids(i).position(2)+1.5];
                pos = [boids(i).position(1)-1 boids(i).position(2)-1 5 5];
                obj.boids_figure_handles(i) =  patch(x,y,'k');
                
                
                
%                 obj.boids_figure_handles(i) = rectangle('Position', pos, 'Curvature', [1 1],...
%                     'FaceColor',[0 0 0]);
            end
        end
        
    end
    
end

