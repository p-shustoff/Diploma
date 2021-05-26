classdef Charging_Station
    properties
        Node
        Available_Chargers
        Total_Chargers
        Power
    end
    
    methods
        
        function obj = Charging_Station(Node,Total_Chargers,Power)
            obj.Node = Node;
            obj.Total_Chargers = Total_Chargers;
            obj.Available_Chargers = obj.Total_Chargers;
            obj.Power = Power;
        end
        
        function [obj,state] = take_charger(obj)
            if (obj.Available_Chargers > 0)
                obj.Available_Chargers = obj.Available_Chargers - 1;
                state = 1;
            else 
                state = 0;
            end
        end
            
        function [obj ,state] = set_free(obj)
            k = obj.Total_Chargers - obj.Available_Chargers;
            if (k > 0) 
                obj.Available_Chargers = obj.Available_Chargers + 1;
                state = 1;
            else
                state = 0;
            end
        end
    end            
end