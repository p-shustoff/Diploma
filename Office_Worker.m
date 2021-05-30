classdef Office_Worker < EV
    properties
        work_Node
        trajectory
        in = 0;
        in_home_office = 0;
        start_charging
    end
    
    methods
        function obj = Office_Worker(mean_V, Graph, all_nodes,SOC_max)
            obj@EV(mean_V, Graph, all_nodes,SOC_max)
            obj.departures = [normrnd(7, 1/3) normrnd(17, 1/3)];
            obj.state = "home";
            index_home = find(all_nodes == obj.home_Node);
            all_nodes(index_home) = [];
            shuffl = randperm(length(all_nodes));
            all_nodes = all_nodes(shuffl);
            index_work = randi([1, length(all_nodes)],1);
            obj.work_Node = all_nodes(index_work);
            obj.from_Node = obj.home_Node;
            obj.trajectory = shortestpath(Graph,obj.home_Node,obj.work_Node);
            obj.to_Node = obj.trajectory(2);
        end
        
        function obj = set_state(obj,time)
            if(time < obj.departures(1) || (time >= obj.departures(2) &&  obj.from_Node == obj.home_Node))
                obj.state = "home";
            elseif (time >= obj.departures(1) &&  time < obj.departures(2) && obj.from_Node ~= obj.work_Node)
                obj.state = "driving_to_office";
            elseif (time >= obj.departures(2) && obj.from_Node ~= obj.home_Node)
                obj.state = "driving_home";
                obj.in_home_office = 0;
            else
                obj.state = "office";
            end
        end
        
        function [obj,Station_massive] = move_and_charge(obj,x_target,y_target,Graph, station_Nodes, Station_massive, curr_time)
            step = 0.002;
            if (startsWith(obj.state,"driving_"))
                step = 0.002;
                old_x = obj.x_coord;
                old_y = obj.y_coord;
                old_SOC = obj.SOC;
                dx_angle = x_target - old_x; 
                dy_angle = y_target - old_y;
                cond = abs(sqrt((dx_angle)^2 + (dy_angle)^2));
                if (cond <= 0.07)
                   if (startsWith(obj.state,"driving_"))
                      if (obj.state == "driving_home")
                        trajectory = shortestpath(Graph, obj.to_Node,obj.home_Node);
                        obj = drive_to_trajectory(obj,trajectory,x_target,y_target,"home");
                      elseif (obj.state == "driving_to_station")
                        distances = [];
                        for p = 1:length(station_Nodes)
                            [~,d] = shortestpath(Graph,obj.to_Node,station_Nodes(p));
                            distances(p) = d;
                        end
                        mindist = min(distances);
                        min_index = find(distances == mindist);
                        obj.Station_pos = min_index;
                        trajectory = shortestpath(Graph,obj.to_Node,station_Nodes(min_index));
                        obj = drive_to_trajectory(obj,trajectory,x_target,y_target,"charging_st");
                        obj.wasted_time = obj.wasted_time + step;
                      elseif (obj.state == "driving_to_office")
                        trajectory = shortestpath(Graph, obj.to_Node,obj.work_Node);
                        obj = drive_to_trajectory(obj,trajectory,x_target,y_target,"office");
                      end
                   end
                else
                    sigx = sign(dx_angle);
                    sigy = sign(dy_angle);
                    otnosh = abs(dx_angle/dy_angle);
                    if (otnosh == Inf)
                        V_x = sigx * obj.V;
                        V_y = 0;
                    elseif (otnosh == 0)
                        V_x = 0;
                        V_y = sigy * obj.V;
                    else
                        V_y = sigy * sqrt((obj.V^2)/((otnosh^2)+1));
                        V_x = sigx * abs(otnosh * V_y);
                    end
                    obj.x_coord = old_x + step*V_x;
                    obj.y_coord = old_y + step*V_y;
                    obj.SOC = old_SOC - step * 10;
                    tmp_SOC = obj.SOC;
                    if (obj.SOC < 4.6833 && obj.state ~= "driving_home")
                        obj.state = "driving_to_station";
                    end
                end
            elseif (obj.state == "home")
                old_SOC = obj.SOC;
                old_x = obj.x_coord;
                old_y = obj.y_coord;
                obj.x_coord = old_x;
                obj.y_coord = old_y;
%                 if (obj.in_home_office == 0 && (curr_time > obj.departures(2)))
%                    prob = betarnd(1.1,3);
%                    obj.start_charging = (24 - curr_time) * prob + curr_time;
%                    obj.in_home_office = 1;
%                 end
                if ((obj.SOC < obj.SOC_max)) % && curr_time < obj.departures(1)) || ((obj.SOC < obj.SOC_max && curr_time > obj.departures(2)) && (curr_time > obj.start_charging)))
                    obj.SOC = old_SOC + step * 1.5;
                end
            elseif (obj.state == "charging_st")
                if (obj.in == 0)
                    [Station_massive{obj.Station_pos},state]= Station_massive{obj.Station_pos}.take_charger;
                    if (state == 1)
                        obj.in = 1;
                    elseif(state == 0)
                        obj.in = 2;
                    end
                end
                old_SOC = obj.SOC;
                old_x = obj.x_coord;
                old_y = obj.y_coord;
                obj.x_coord = old_x;
                obj.y_coord = old_y;
                if (obj.in == 1)
                    obj.SOC = old_SOC + step * Station_massive{obj.Station_pos}.Power;
                elseif (obj.in == 2)
                    obj.in = 0;
                end
                if (obj.SOC > 0.60 * obj.SOC_max)
                    obj.state = "medium";
                    Station_massive{obj.Station_pos} = Station_massive{obj.Station_pos}.set_free;
                    obj.in = 0;
                end
            elseif (obj.state == "office")
                old_SOC = obj.SOC;
                old_x = obj.x_coord;
                old_y = obj.y_coord;
                obj.x_coord = old_x;
                obj.y_coord = old_y;
                if (obj.in_home_office == 0)
                   req_time = (0.75 * obj.SOC_max - obj.SOC) / 3;
                   max_start = obj.departures(2) - req_time;
                   if (max_start > curr_time)
                       obj.start_charging = randinter(curr_time, max_start);
                   else
                        obj.start_charging = curr_time;
                   end
                   obj.in_home_office = 1;
                end 
                if (obj.SOC < obj.SOC_max && curr_time >= obj.start_charging)
                    obj.SOC = old_SOC + step * 3;
                end
            else
                error("NO WAY!")
            end
        end
        
        function obj = drive_to_trajectory(obj,trajectory,x_target,y_target,next_state)
            toNode = obj.to_Node;
            obj.from_Node = toNode;
            obj.trajectory = trajectory;
            if (length(obj.trajectory) > 1)
                obj.to_Node = obj.trajectory(2);
            else
                obj.state = next_state;
            end
            obj.x_coord = x_target;
            obj.y_coord = y_target;
        end
    end
end