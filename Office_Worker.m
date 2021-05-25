classdef Office_Worker < EV
    properties
        home_Node
        work_Node
        trajetory
    end
    
    methods
        function obj = Office_Worker(mean_V, Graph, all_nodes)
            obj@EV(mean_V, Graph, all_nodes)
            obj.departures = [normrnd(7, 1/3) normrnd(17, 1/3)];
            obj.state = "home";
            all_nodes(index_home) = [];
            shuffl = randperm(length(all_nodes));
            all_nodes = all_nodes(shuffl);
            index_work = randi([1, length(all_nodes)],1);
            obj.work_Node = all_nodes(index_work);
            obj.from_Node = obj.home_Node;
            obj.trajectory = shortestpath(Graph,obj.home_Node,obj.work_Node);
            obj.to_Node = obj.trajectory(2);
            obj.trajectory(1) = [];
        end
        
        function obj = set_state(obj,time)
            if(time < obj.departures(1) || (time >= obj.departures(2) &&  obj.from_Node == obj.home_Node))
                obj.state = "home";
            elseif((time >= obj.departures(1) && obj.from_Node ~= obj.work_Node) || ...
                   (time >= obj.departures(2) && obj.from_Node ~= obj.home_Node))
                obj.state = "driving";
            else
                obj.state = "office";
            end
        end
        
        function obj = move_and_charge(obj,x_target,y_target,Graph)
            step = 0.002;
            if (obj.state == "driving")
                step = 0.002;
                old_x = obj.x_coord;
                old_y = obj.y_coord;
                old_SOC = obj.SOC;
                dx_angle = x_target - old_x; 
                dy_angle = y_target - old_y;
                cond = abs(sqrt((dx_angle)^2 + (dy_angle)^2));
                if (cond <= 0.07)
                    toNode = obj.to_Node;
                    obj.to_Node = obj.trajectory(2);
                    obj.trajectory(1) = [];
                    obj.x_coord = x_target;
                    obj.y_coord = y_target;
                    obj.from_Node = toNode;
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
                    if (obj.SOC < 15)
                        obj.state = "charging_st";
                    end
                end
            elseif (obj.state == "home")
                old_SOC = obj.SOC;
                old_x = obj.x_coord;
                old_y = obj.y_coord;
                obj.x_coord = old_x;
                obj.y_coord = old_y;
                obj.SOC = old_SOC + step * 3;
            elseif (obj.state == "charging_st")
                old_SOC = obj.SOC;
                old_x = obj.x_coord;
                old_y = obj.y_coord;
                obj.x_coord = old_x;
                obj.y_coord = old_y;
                obj.SOC = old_SOC + step * 2;
            elseif (obj.state == "office")
                old_SOC = obj.SOC;
                old_x = obj.x_coord;
                old_y = obj.y_coord;
                obj.x_coord = old_x;
                obj.y_coord = old_y;
                obj.SOC = old_SOC + step * 1;
            else
                error("NO WAY!")
            end
        end
    end
end