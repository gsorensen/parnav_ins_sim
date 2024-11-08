classdef SensorReadingFault
    %SENSORREADING Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Out of 1000
        ChanceOfNoOutput
        ChanceOfNullReading
        ChanceOfRepeatedReading
        ChanceOfAbnormalErrorSpike
        ChanceOfAbornmalErrorMP
    end
    
    methods
        function obj = SensorReadingFault(ChanceOfNoOutput1000,...
                                     ChanceOfNullReading1000,...
                                     ChanceOfRepeatedReading1000,...
                                     ChanceOfAbnormalErrorSpike1000,...
                                     ChanceOfAbnormalErrorMP1000)
            obj.ChanceOfNoOutput = ChanceOfNoOutput1000;
            obj.ChanceOfNullReading = ChanceOfNullReading1000;
            obj.ChanceOfRepeatedReading = ChanceOfRepeatedReading1000;
            obj.ChanceOfAbnormalErrorSpike = ChanceOfAbnormalErrorSpike1000;
            obj.ChanceOfAbornmalErrorMP = ChanceOfAbnormalErrorMP1000;
            % Assign a (low) integer value to each type of error. Each
            % iteration, it will do a randni(1000) <= number check to see
            % if it should trigger a fault at that point. 
            % Currently exclusive, only one error may occur, but e.g.,
            % abornmal error could in theory occur with repeated reading,
            % and you could get a repeated reading of a null or nan.
        end
        
        function no = is_no_output(obj)
            if randi(1000) <= obj.ChanceOfNoOutput
                no = 1;
            else
                no = 0;
            end
        end

        function nr = is_null_reading(obj)
            if randi(1000) <= obj.ChanceOfNullReading
                nr = 1;
            else
                nr = 0;
            end
        end

        function rr = is_repeated_reading(obj)
            if randi(1000) <= obj.ChanceOfRepeatedReading
                rr = 1;
            else
                rr = 0;
            end
        end

        function ae = is_spike(obj)
            if randi(1000) <= obj.ChanceOfAbnormalErrorSpike
                ae = 1;
            else
                ae = 0;
            end
        end

        function ae = is_multipath(obj)
            if randi(1000) <= obj.ChanceOfAbornmalErrorMP
                ae = 1;
            else
                ae = 0;
            end
        end
    end
end

