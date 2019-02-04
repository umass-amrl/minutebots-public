D = dlmread('../../latency.txt');
l = D(:,3) - D(:,2);
plot(l(1:100),'*')
