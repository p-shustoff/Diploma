clc
clf
clear all
mean_collect = 1;

% Подготовка рабочей области и подгрузка графа

load('mean_data.mat');
load('graph.mat');
load('traffic.mat');
time = single([0:0.002:24]);

% Подргузка графика нагрузки

load('residential_load.mat');
subplot(2,2,4)
loadprof = plot(time, P_n);

% Задание города в виде графа 

f = figure(1);
f.Position = [100 100 600 700];
subplot(2,2,[1 3])
h = plot(G, 'XData', test_x, 'YData', test_y);
title("Модель движения электромобилей")
xlabel("x, км")
ylabel("y, км")
hold on
Graph_size = int16(size(G.Nodes));
all_nodes  = int16([1:Graph_size(1)]);

% Зарядные станции, узлы зарядных станций

Station1 = Charging_Station(449,10,50);
Station2 = Charging_Station(548,10,50);
Station3 = Charging_Station(375,10,50);
Station4 = Charging_Station(327,10,50);
Station5 = Charging_Station(513,10,50);
Station6 = Charging_Station(548,10,50);
Station7 = Charging_Station(303,10,50);

station_Nodes = [449,548,375,327,513,438,303];

station_massive = {Station1, Station2, Station3, Station4, Station5, Station6, Station7};

% Электромобили

N_tot = 5000;

N_off = 0.5 * N_tot;

N_dr = 0.4 * N_tot;

N_ndr = 0.1 * N_dr;

N_home = 0.1 * N_tot; 

EV_arr = {};
EV_arr2 = {};
EV_arr3 = {};
EV_arr4 = {};

for i = 1 : N_off
    EV_arr{i} = Home_Worker(50,G,all_nodes,24);
    EV_arr{i}.x_coord = test_x(EV_arr{i}.from_Node);
    EV_arr{i}.y_coord = test_y(EV_arr{i}.from_Node);
end

for i = 1 : N_dr
    EV_arr2{i} = Driver(50,G,all_nodes,48);
    EV_arr2{i}.x_coord = test_x(EV_arr2{i}.from_Node);
    EV_arr2{i}.y_coord = test_y(EV_arr2{i}.from_Node);
end

for i = 1 : N_ndr
    EV_arr3{i} = Night_Driver(50,G,all_nodes,48);
    EV_arr3{i}.x_coord = test_x(EV_arr3{i}.from_Node);
    EV_arr3{i}.y_coord = test_y(EV_arr3{i}.from_Node);
end

for i = 1 : N_home
    EV_arr4{i} = Office_Worker(50,G,all_nodes,48);
    EV_arr4{i}.x_coord = test_x(EV_arr4{i}.from_Node);
    EV_arr4{i}.y_coord = test_y(EV_arr4{i}.from_Node);
end


 for i = 1:N_dr
    EV_arr(N_off+i) = EV_arr2(i); 
 end  
 
 for j = 1:N_ndr
    EV_arr(N_off+N_dr+j) = EV_arr3(j); 
 end  
 
 for k = 1:N_home
     EV_arr(N_off+N_dr+N_ndr+k) = EV_arr4(k);
 end

b = 1;
v = 1;
c = 1;
y = 1;

if ( mean_collect == 1)
    for z = 1:length(EV_arr)
       if ( isa(EV_arr{z},'Driver') && (1 - isa(EV_arr{z},'Night_Driver')))
            EV_arr{z}.SOC = driver_SOC(b);
            b = b + 1; 
       elseif ( isa(EV_arr{z},'Driver') &&  isa(EV_arr{z},'Night_Driver'))
            EV_arr{z}.SOC =  nightdriver_SOC(v); 
            v = v + 1;
       elseif (isa(EV_arr{z},'Office_Worker'))
            EV_arr{z}.SOC = office_SOC(c);
            c = c + 1;
       elseif ( isa(EV_arr{z},'Home_Worker'))
            EV_arr{z}.SOC = home_SOC(y);
            y = y + 1;  
       end
    end
end

EV_arr = EV_arr(randperm(numel(EV_arr)));

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
subplot(2,2,2)
powerpl = plot(time(1:end-1),power);
axis([0 24 0 5*N]);
title("Потребляемая из сети мощность")
xlabel("t, ч")
ylabel("P, кВт*ч")
velocity_over_time = [];


% Главный цикл анимации

for i = 1:length(time)
    
     traffic_point = traffic(i);
    
     for j = 1:N
       EV_arr{j} = EV_arr{j}.set_state(time(i));
       EV_arr{j} = EV_arr{j}.traffic_speed(traffic_point);
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
       [EV_arr{k},station_massive] = EV_arr{k}.move_and_charge(x_target,y_target,G,station_Nodes,station_massive,time(i));
     end
     
%         EV_arr{1}
     
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

subplot(2,2,4)
hold on
total_load = plot(time(1:end-1), P_n(1:end-1) + (power * 10e-3)');


figure(3)

plot(time, EV_arr{1}.V * velocity_over_time) 

if (mean_collect == 0)
    driver_SOC = [];
    nightdriver_SOC = [];
    office_SOC = [];
    home_SOC = [];
    b = 1;
    v = 1;
    c = 1;
    y = 1;
    z = 1;
    
    for z = 1:length(EV_arr)
       if ( isa(EV_arr{z},'Driver') && (1 - isa(EV_arr{z},'Night_Driver')))
            driver_SOC(b) = EV_arr{z}.SOC;
            b = b + 1; 
       elseif ( isa(EV_arr{z},'Driver') &&  isa(EV_arr{z},'Night_Driver'))
            nightdriver_SOC(v) = EV_arr{z}.SOC; 
            v = v + 1;
       elseif ( isa(EV_arr{z},'Office_Worker') )
            office_SOC(c) = EV_arr{z}.SOC;
            c = c + 1;
       elseif ( isa(EV_arr{z},'Home_Worker') )
            home_SOC(y) = EV_arr{z}.SOC;
            y = y + 1;  
       end
    end
    save ('mean_data.mat','driver_SOC','nightdriver_SOC','office_SOC','home_SOC');
end

