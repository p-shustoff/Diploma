% Подготовка рабочей области и подгрузка графа

clc
clf
clear all
load('graph.mat');
time = single([0:0.002:24]);

% Задание города в виде графа 

f = figure(1);
f.Position = [100 100 600 700];
subplot(2,1,1)
h = plot(G, 'XData', test_x, 'YData', test_y);
title("Модель движения электромобилей")
hold on
Graph_size = int16(size(G.Nodes));
all_nodes  = int16([1:Graph_size(1)]);

% Зарядные станции, узлы зарядных станций

Station1 = Charging_Station(449,10,8);
Station2 = Charging_Station(548,10,8);
Station3 = Charging_Station(375,10,8);
Station4 = Charging_Station(327,10,8);
Station5 = Charging_Station(513,10,8);
Station6 = Charging_Station(548,10,8);
Station7 = Charging_Station(303,10,8);

station_Nodes = [449,548,375,327,513,438,303];

station_massive = {Station1, Station2, Station3, Station4, Station5, Station6, Station7};

% Электромобили

N_dr = 100;
% N_ndr = 0.1 * N_dr;

EV_arr = {};
% EV_arr2 = {};

for i = 1 : N_dr
    EV_arr{i} = Office_Worker(50,G,all_nodes,24);
    EV_arr{i}.x_coord = test_x(EV_arr{i}.from_Node);
    EV_arr{i}.y_coord = test_y(EV_arr{i}.from_Node);
end

% for i = 1 : N_ndr
%     EV_arr2{i} = Night_Driver(50,G,all_nodes,48);
%     EV_arr2{i}.x_coord = test_x(EV_arr2{i}.from_Node);
%     EV_arr2{i}.y_coord = test_y(EV_arr2{i}.from_Node);
% end

% for i = 1:N_ndr
%    EV_arr(N_dr+i) = EV_arr2(i); 
% end  

% EV_arr{1}.departures = [12 3];
% EV_arr{1}.SOC = 10;

% EV_arr = EV_arr(randperm(numel(EV_arr)));

N = numel(EV_arr);

x_vect = zeros(N,1);
y_vect = x_vect;

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

velocity_over_time = [];

% Главный цикл анимации

for i = 1:length(time)
    
     for j = 1:N
       EV_arr{j} = EV_arr{j}.set_state(time(i));
     end
    
     if (startsWith(EV_arr{1}.state, "driving"))
         velocity_over_time(i) = 1;
     else
         velocity_over_time(i) = 0;
     end

     for o = 1:N
         SOC_prev(o) = EV_arr{o}.SOC;
     end

     for k = 1:N   
       x_target = test_x(EV_arr{k}.to_Node);
       y_target = test_y(EV_arr{k}.to_Node);
       [EV_arr{k},station_massive] = EV_arr{k}.move_and_charge(x_target,y_target,G,station_Nodes,station_massive);
     end
     
%      EV_arr{1}
     
     for s = 1:N
        delta_p(s) = EV_arr{s}.SOC - SOC_prev(s);
        if(delta_p(s) < 0)
            delta_p(s) = 0;
        end
     end
        
     if (i < length(time))
        power(i) = sum(delta_p)/0.002;
     end
     
     for l=1:N
          x_vect(l) = EV_arr{l}.x_coord;
          y_vect(l) = EV_arr{l}.y_coord;
     end

      curr_time = num2str(time(i));
      legend("t = " + curr_time);
      set(h, 'XData', x_vect, 'YData', y_vect);
      set(powerpl,'XData', time(1:end-1), 'YData', power);
      pause(0.005)
end

figure(3)

plot(time, EV_arr{1}.V * velocity_over_time) 

