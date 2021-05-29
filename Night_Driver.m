classdef Night_Driver < Driver
    
    methods
        function obj = Night_Driver(mean_V, Graph, all_nodes,SOC_max)
                G_size = size(Graph.Nodes);
                obj@Driver(mean_V, Graph, all_nodes,SOC_max);
                obj.departures = [normrnd(20, 2/3) normrnd(7, 2/3)];
                obj.state = "driving";
                from_Node = randi([1, G_size(1)],1);
                obj.from_Node = from_Node;
                neighb = neighbors(Graph,obj.from_Node);
                index = randi([1, length(neighb)],1);
                obj.to_Node = neighb(index);
        end
        
        function obj = set_state(obj,time)
             if((time < obj.departures(1) && time >= obj.departures(2)) &&  obj.from_Node == obj.home_Node && obj.wasted_time < 1e-10)
                obj.state = "home";
             elseif ((time >= obj.departures(2) && time < obj.departures(1)) &&  obj.from_Node ~= obj.home_Node && obj.state ~="charging_st" && obj.wasted_time < 1e-10)
                obj.state = "driving_home";
             else
                if (xor((startsWith(obj.state,"driving_")), (obj.state ~= "charging_st")))
                    obj.state = "driving";
                end
             end
        end
    end
end