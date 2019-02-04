DlgH = figure;
H = uicontrol('Style', 'PushButton', ...
                    'String', 'Break', ...
                    'Callback', 'delete(gcbf)');
kLength = 400;
while (ishandle(H))
  D = dlmread('../data.txt');
  N = size(D, 1);
  n_start = max(1, N-kLength);
  D = D(n_start:end,:);
  plot(D, 'linewidth', 2);
  axis([0 min(N, kLength) -1600 1600]);
  legend('P','I','D','E');
  mean_error = mean(D(:, 4)) * 100;
  % fprintf('Error:%d\r', mean_error);
  usleep(2000);
end
