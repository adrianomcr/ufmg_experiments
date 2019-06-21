

% M = dlmread('./../text/results_quadrado.txt'); delta = 20*pi/180;
M = dlmread('./../text/results_oito.txt'); delta = -80*pi/180;
% M = dlmread('./../text/results_circulo.txt'); delta = -120*pi/180;


x = M(:,1);
y = M(:,2);
yaw = M(:,8) + delta;


[B,A] = butter(2,0.1);
x = filtfilt(B,A,x);
y = filtfilt(B,A,y);

plot(x,y,'r');
axis equal
axis ([-1 1 -1 1]*7);
grid on


ROB0 = [[0.3 -0.2 -0.1 -0.2 0.3];[0 0.4 0 -0.4 0]]*0.5;
R = [cos(yaw(1)) -sin(yaw(1)); sin(yaw(1)) cos(yaw(1))];
ROB = R*ROB0 + [x(1);y(1)]*ones(1,5);

hold on
% h1 = plot(x(1),y(1),'bo','MarkerSize',20);
% h2 = plot(ROB0(1,:),ROB0(2,:),'-b','LineWidth',2);
h2 = fill(ROB(1,:),ROB(2,:),'-b');
hold off
for k = 1:1:length(x)
%     yaw = 0.2;
    R = [cos(yaw(k)) -sin(yaw(k)); sin(yaw(k)) cos(yaw(k))];
    ROB = R*ROB0 + [x(k);y(k)]*ones(1,5);
    
%     set(h1,'XData',x(k),'YData',y(k));
    set(h2,'XData',ROB(1,:),'YData',ROB(2,:));
    
    drawnow
    
end



