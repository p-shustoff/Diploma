classdef EV
   
    properties
        x_coord
        y_coord
        V
        from_Node
        to_Node
        departures
        SOC
        state
    end
      
    methods
        
        function obj = EV(mean_V, Graph, all_nodes)
            G_size = size(Graph.Nodes);
            index_home = randi([1, length(all_nodes)],1);
            obj.home_Node = all_nodes(index_home);
            obj.V = normrnd(mean_V, 10/3);
            obj.SOC = randinter(50,100);
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
                    neighbours = neighbors(Graph,toNode);
                    index = find(neighbours == obj.from_Node);
                    if (length(neighbours) > 1)
                        neighbours(index) = [];
                    end
                    index = randi([1, length(neighbours)],1);
                    obj.to_Node = neighbours(index);
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