%% Simulación de Boids de Reynolds
%% Simulación
boids_count=15;
predator_count=1;
predator=Predator.empty;
boids=Boid.empty;
predator=Predator(200,200)
for i=1:boids_count
   boids(i)=Boid(rand*380,rand*480,predator);
end
tic
flock=Flock(boids,[350 450],predator);%640/3 360/3
trad = flock.ftimes
fprintf('Tiempos:');
            
%tEnd = toc(tStart)
%ftime=toc
%fprintf('Tiempo transcurrido: %.4f segundos\n', ftime);
boids = boids;
f = figure;
plane = Plane(f,[350 450],boids,predator);
flock.run(plane);
