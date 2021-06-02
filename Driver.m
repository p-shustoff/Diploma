classdef Driver < EV
    properties
        trajectory
        Station_pos
        in = 0;
        wasted_time = 0;
        
    end
    methods
        function obj = Driver(mean_V, Graph, all_nodes, SOC_max)
            obj@EV(mean_V, Graph, all_nodes, SOC_max)
            obj.departures = [normrnd(9, 2/3) normrnd(18, 2/3)];
            obj.state = "home";
            obj.from_Node = obj.home_Node;
            neighb = neighbors(Graph,obj.from_Node);
            index = randi([1, length(neighb)],1);
            obj.to_Node = neighb(index);
        end
        
        function obj = set_state(obj,time)
             if(time < obj.departures(1) || (time >= obj.departures(2) &&  obj.from_Node == obj.home_Node && obj.wasted_time < 1e-10))
                obj.state = "home";
             elseif (time >= obj.departures(2) &&  obj.from_Node ~= obj.home_Node && obj.state ~="charging_st" && obj.wasted_time < 1e-10)
                obj.state = "driving_home";
             else
                if (xor((startsWith(obj.state,"driving_")), (obj.state ~= "charging_st")))
                    obj.state = "driving";
                end
             end
        end
        
        function [obj,Station_massive] = move_and_charge(obj,x_target,y_target,Graph, station_Nodes, Station_massive,curr_time)
            step = 0.002;
            if (obj.state == "driving" || startsWith(obj.state,"driving_"))
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
                            distances = zeros(1,length(station_Nodes));
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
                        end
                    else
                        toNode = obj.to_Node;
                        neighbours = neighbors(Graph,toNode);
                        index = find(neighbours == obj.from_Node);
                        if (length(neighbours) > 1)
                            neighbours(index) = [];
                        end
                        if (obj.wasted_time > 0)
                            obj.wasted_time = obj.wasted_time - step;
                        end
                        index = randi([1, length(neighbours)],1);
                        obj.to_Node = neighbours(index);
                        obj.x_coord = x_target;
                        obj.y_coord = y_target;
                        obj.from_Node = toNode;
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
                    if (obj.state == "driving" && obj.wasted_time > 0)
                        obj.wasted_time = obj.wasted_time - step;
                    elseif (obj.state == "driving_to_station")
                        obj.wasted_time = obj.wasted_time + step;
                    end
                    obj.x_coord = old_x + step*V_x;
                    obj.y_coord = old_y + step*V_y;
                    obj.SOC = old_SOC - (obj.SOC_max * obj.V / 400) * step;
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
                if (obj.SOC < obj.SOC_max)
                    obj.SOC = old_SOC + step * 3.5;
                end
            elseif (obj.state == "charging_st")
                obj.wasted_time = obj.wasted_time + step;
                if (obj.in == 0)
                    [Station_massive{obj.Station_pos},state]= Station_massive{obj.Station_pos}.take_charger;
                    if (state == 1)
                        obj.in = 1;
                    elseif(state == 0);
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
                    if (isa(obj,'Night_Driver'))
                        obj.departures = [obj.departures(1)+obj.wasted_time obj.departures(2)];
                    end
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