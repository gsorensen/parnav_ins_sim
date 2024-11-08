classdef SensorFaultSimulator
    %SENSORFAULTSIMULATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        SensorFaultChances
        Mean
        StandardDeviation
        ErrorCount
        ErrorKind % 0 spike, 1 zero, 2 sinus mix
    end
    
    methods
        function obj = SensorFaultSimulator(DistributionMean, ...
                                            DistributionStandardDeviation,...
                                            ChanceOfNoOutput1000,...
                                            ChanceOfNullReading1000,...
                                            ChanceOfRepeatedReading1000,...
                                            ChanceOfAbnormalErrorSpike1000, ...
                                            ChanceOfAbnormalErrorMP1000 ...
                                            )
            obj.Mean = DistributionMean;
            obj.StandardDeviation = DistributionStandardDeviation;
            obj.SensorFaultChances = SensorReadingFault(ChanceOfNoOutput1000,...
                                     ChanceOfNullReading1000,...
                                     ChanceOfRepeatedReading1000,...
                                     ChanceOfAbnormalErrorSpike1000, ...
                                     ChanceOfAbnormalErrorMP1000);
        end
        
        %function [sensor_error, error_count, error_indices, error_descriptions] = roll_errors(obj, measurements)
        function [sensor_error, error_flags] = roll_errors(obj, measurements)
            N = size(measurements, 2);

            % For storing the outputes
            sensor_error = zeros(size(measurements));  
            
            % Error flags
            error_flags = zeros(size(measurements));
            
            % Spawn the 2nd order GM processes to be used for multipath
            % Noise parameters not settled.
            dt = 0.1; % Should probably propagate through code
            xi = 0.9; 
            w_n = 0.9;
            q = 0.5; 
            tau = 0.3;
            P0 = q^2 .* eye(2,2);


            is_multipath_triggered = false;
            min_multipath_samples = 3;
            max_multipath_samples = 15;
            num_samples = 3;
            sample_count = 0;
            
            Nt = floor(N/10);


            gmt = SecondOrderGaussMarkov(dt,xi,w_n,q,tau,P0,Nt);
            gmstore = zeros(1,Nt) * NaN;
            counter = 2;

            seqs = [];
        
            for i = 2 : N
                % Propagate the Gauss-Markov whether there is
                % a measurement step.

                if size(nonzeros(measurements(:,i)),1) > 0
                    gmt.propagate(counter);

                    % If we're currently in mulitpath, do not roll errors
                    % and just continue generating samples
                    if is_multipath_triggered == true 
                        gm.propagate(counter);
                        sensor_error(2, i) = gmt.x(1,counter);
                        gmstore(counter) = gm.x(1,counter);
                        % This assumes 3d PARS vector
                        error_flags(2, i) = 1;
                        sample_count = sample_count + 1;
                        fprintf("Generating sample %d at %i\n", sample_count, i);

                        % Reset the counter
                        if sample_count == num_samples
                            fprintf("Generation done. Resetting\n");
                            is_multipath_triggered = false;
                            sample_count = 0;
                        end
                    elseif obj.SensorFaultChances.is_null_reading() == 1 
                        sensor_error(:,i) = -measurements(:,i);
                        error_flags(:, i) = 5;
                    elseif obj.SensorFaultChances.is_no_output() == 1
                        sensor_error(:,i) = NaN;
                        error_flags(:, i) = 3;
                    elseif i > 1 && obj.SensorFaultChances.is_repeated_reading() == 1
                        sensor_error(:,i) = measurements(:,i - 1);
                        error_flags(:, i) = 4;
                    elseif obj.SensorFaultChances.is_spike() == 1
                        sensor_error(:, i) = 2 .* obj.StandardDeviation .* randn(size(measurements(:, i)));
                        error_flags(:, i) = 2;
                    elseif obj.SensorFaultChances.is_multipath() == 1
                        is_multipath_triggered = true;
                        num_samples = randi([min_multipath_samples max_multipath_samples], 1);
                        seqs = [seqs num_samples];
                        gm = SecondOrderGaussMarkov(dt,xi,w_n,q,tau,P0,N);

                        fprintf("Multipath triggered for %d samples for locator at idx %d\n", num_samples, i);
                    end
                    counter = counter + 1;
                end
            end
            %{
            figure(1111); clf; hold on;
            histogram(seqs);

            figure(117); clf; hold on;
            subplot(211); hold on;
            plot(gmt.x(1,:));
            subplot(212); hold on;
            plot(gmstore);
            %}
            %legend(["Long lived" "Initialise and discard each trigger"])
        end
    end
end

