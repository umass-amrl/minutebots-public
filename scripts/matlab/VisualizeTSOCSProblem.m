function VisualizeTSOCSProblem(D)
x0 = D(1:2);
xf = D(5:6);
v0 = D(3:4)
vf = D(7:8)

clf;
hold on;
dot_size = [100 100];
rectangle('position', [x0 - 0.5 * dot_size, dot_size],...
          'facecolor',[1 0 0],...
          'Curvature',[1 1]);
rectangle('position', [xf - 0.5 * dot_size, dot_size],...
          'facecolor',[1 0 0],...
          'Curvature',[1 1]);
quiver(x0(1),x0(2), v0(1),v0(2));
quiver(xf(1),xf(2), vf(1),vf(2));
axis equal
hold off;
grid;
end