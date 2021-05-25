clear all
load('graph.mat');
time = single([0:0.002:24]);

% Задание города в виде графа 

f = figure(1);
f.Position = [100 100 600 700];
subplot(2,1,1)
plot(G, 'XData', test_x, 'YData', test_y)
title("Модель движения электромобилей")
hold on
Graph_size = int16(size(G.Nodes));
all_nodes  = int16([1:Graph_size(1)]);

% Электромобили

N = 1000;

EV_arr = {};

for i = 1 : N
    EV_arr{i} = Night_Driver(50,G,all_nodes);
    EV_arr{i}.x_coord = test_x(EV_arr{i}.from_Node);
    EV_arr{i}.y_coord = test_y(EV_arr{i}.from_Node);
end

% EV1 = Night_Driver(50,G,all_nodes);
% EV1 = EV(50,G,"driver");
% EV2 = EV(50,G,"office");
% EV2 = Night_Driver(50,G,all_nodes);

% EV1.x_coord = test_x(EV1.from_Node); EV1.y_coord = test_y(EV1.from_Node);
% EV2.x_coord = test_x(EV2.from_Node); EV2.y_coord = test_y(EV2.from_Node);

x_vect = zeros(N,1);
y_vect = x_vect;

% x_vect = [EV1.x_coord EV2.x_coord];
% y_vect = [EV1.y_coord EV2.y_coord];

for i=1:N
    x_vect(i) = EV_arr{i}.x_coord;
    y_vect(i) = EV_arr{i}.y_coord;
end

h = plot(x_vect, y_vect , 'ro','MarkerSize',2);

power = zeros(length(time)-1,1);
power(2:end) = NaN;
subplot(2,1,2)
powerpl = plot(time(1:end-1),power);
axis([0 24 0 5*N]);



% subplot(2,1,2)
% 
% SOC = zeros(length(time),1);
% SOC(2:end) = NaN;
% 
% hh =  plot(time, SOC);
% axis([0 time(end) 0 120]);

for i = 1:length(time)
    
     for j = 1:N
       EV_arr{j} = EV_arr{j}.set_state(time(i));
     end
%      
     for o = 1:N
         SOC_prev(o) = EV_arr{o}.SOC;
     end
%       EV2 = EV2.set_state(time(i));
     for k = 1:N   
       x_target = test_x(EV_arr{k}.to_Node);
       y_target = test_y(EV_arr{k}.to_Node);
       EV_arr{k} = EV_arr{k}.move_and_charge(x_target,y_target,G);
     end
%       x_target2 = test_x(EV2.to_Node);
%       y_target2 = test_y(EV2.to_Node); 
     
     for s = 1:N
        delta_p(s) = EV_arr{s}.SOC - SOC_prev(s);
        if(delta_p(s) < 0)
            delta_p(s) = 0;
        end
     end
        
     if (i < length(time))
        power(i) = sum(delta_p);
     end
     
     for l=1:N
          x_vect(l) = EV_arr{l}.x_coord;
          y_vect(l) = EV_arr{l}.y_coord;
     end

%     x_target2 = x(EV2.to_Node);
%     y_target2 = y(EV2.to_Node);
%     SOC(i) = EV1.SOC;
      curr_time = num2str(time(i));
      legend("t = " + curr_time);
%     EV2 = EV2.move_and_charge(x_target2,y_target2,G);
      set(h, 'XData', x_vect, 'YData', y_vect);
      set(powerpl,'XData', time(1:end-1), 'YData', power);
%     EV1.to_Node
%     set(hh,'XData',time , 'YData', SOC)
      pause(0.005)
end

