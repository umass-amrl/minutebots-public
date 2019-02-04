clear all;
filename = 'TSOCS-failures-finalV.txt';

D = dlmread(filename);
X0 = D(:, 1:2);
V0 = D(:, 3:4);
Xf = D(:, 5:6);
Vf = D(:, 7:8);

dX = Xf - X0;
dV = Vf - V0;

v_length = zeros(length(dV), 1);
x_length = zeros(length(dV), 1);

for i=1:length(dX)
  x_length(i) = norm(dX(i, :));
  v_length(i) = norm(dV(i, :));
end

v_fraction = abs(sum(dX .* dV, 2))./(v_length.*v_length.*x_length);
x_fraction = abs(sum(dX .* dV, 2))./(v_length.*x_length);

figure(1)
scatter_plot = scatter(x_fraction, v_fraction, '.');
% scatter_plot.MarkerFaceAlpha = .2;
% scatter_plot.MarkerEdgeAlpha = .2;

figure(2)
hist(abs(sum(dX .* dV, 2))./(v_length.*x_length), 40);

% Save TSOCS problems which are not near 1D, but still have failures.
%dlmwrite('TSOCS-failures-non1D.txt', D(dotp<0.001,:), 'precision', '%20f');

%figure(1);
%

%angles = acos(dotp)/pi*180;
%hist(angles, 40);