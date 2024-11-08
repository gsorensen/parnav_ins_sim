function f = plot_INS_error_3sigma_substate(fig_num, plot_title, ...
                                            plot_mask,...
                                            t, ...
                                            xtrue, ...
                                            xestmekf, Pmekf,...
                                            xestfgo, Pfgo,...
                                            xestmekfsf, Pmekfsf)

colour_palette = [
    hex2rgb("009e73"); % blue green
    hex2rgb("cc79a7"); % pale violet 
    hex2rgb("0072b2"); % blue
    hex2rgb("e69f00"); % orange
    hex2rgb("56b4e9"); % sky blue
    hex2rgb("d55e00"); % vermillion
    hex2rgb("f0e442"); % yellow
    hex2rgb("000000"); % black
];

ylabels = ["Position error [m]" , ...
    "Velocity error [m/s]", ...
    "Attitude error [deg]",...
    'Accelerometer bias error [m$/$s$^2$]',...
   "Gyroscope bias error [deg$/$s]"
];



legends = ["ESKF 3std";"ESKF";"FGO 3std"; "FGO"];

if nargin >= 10
    %legends = [legends; "ESKF s.f. 3std";"ESKF s.f."];
    legends = ["ESKF s.f. 3std";"ESKF s.f."; legends];
end

% How many subplots
D = 5;
deg_mask = [0 0 1 0 1];

dx = xtrue - xestmekf;
dxfgo = xtrue - xestfgo;

if nargin >= 10
    dxsf = xtrue - xestmekfsf;
end

f=figure(fig_num); clf; 
numsubplots = sum(plot_mask);
subplotnum = 1;
colour_counter = 1;
substateelementcounter = 1;
N = size(t, 2);

for substatenum = 1 : D

    offset = 3 * (substatenum - 1);
    start_id = 1 + offset;
    end_id = 3 + offset;
    
    if plot_mask(substatenum) == 1
        subplot(numsubplots,1,subplotnum); hold on; grid on;
        

        % For legend to behave as expected, the three substates have to be
        % plotted before we then turn to the error bounds, that's why
        % there are two for-loops
        %for id = start_id : end_id
  
        
        %if axis_mask(substatenum, substateelementcounter) == 1
        normstate = vecnorm(dx(start_id:end_id, :));
        std3vals = zeros(size(normstate));
        normstatefgo = vecnorm(dxfgo(start_id:end_id, :));
        std3valsfgo = zeros(size(normstate));

        if nargin >= 10
            normstatesf = vecnorm(dxsf(start_id:end_id, :));
            std3valssf = zeros(size(normstatesf));
        end

        %v3std=3*reshape(pagenorm(Pmekf(start_id:end_id,start_id:end_id,:), 2), [1 N]);
        for ele = 1 : size(normstate,2)
            std3vals(ele) = 3 * sqrt(trace(Pmekf(start_id:end_id,start_id:end_id,ele)));
            std3valsfgo(ele) = 3 * sqrt(trace(Pfgo(start_id:end_id,start_id:end_id,ele)));

            if nargin >= 10
                std3valssf(ele) = 3 * sqrt(trace(Pmekfsf(start_id:end_id,start_id:end_id,ele)));
            end
        end
        if deg_mask(substatenum) == 1
            normstate = normstate .* (180/pi);
            std3vals = std3vals .* (180/pi);
            normstatefgo = normstatefgo .* (180/pi);
            std3valsfgo = std3valsfgo .* (180/pi);

            if nargin >= 10
                normstatesf = normstatesf .* (180/pi);
                std3valssf = std3valssf .* (180/pi);
            end
            %v3std = v3std .* (180/pi);
        end
        

        %% NOTE TRY SHADING AREA OR MAKING THEM DIFFERENT
        
        if nargin >= 10
        area(t, std3valssf, "FaceColor", colour_palette(1, :), "FaceAlpha", 0.3, "EdgeColor", colour_palette(1, :), "LineStyle", "--");
       % plot(t, std3valssf, "Color", colour_palette(1, :), "LineStyle","--");

        plot(t, normstatesf, "Color", colour_palette(1, :));
       % colour_counter = colour_counter + 1;
        end
        
        area(t, std3vals, "FaceColor", colour_palette(2, :), "FaceAlpha", 0.2, "EdgeColor", colour_palette(2, :), "LineStyle", "--");
%        plot(t, std3vals, "Color", colour_palette(2, :), "LineStyle","--");
        plot(t, normstate, "Color", colour_palette(2, :));

       % colour_counter = colour_counter + 1;
        area(t, std3valsfgo, "FaceColor", colour_palette(3, :), "FaceAlpha", 0.1, "EdgeColor", colour_palette(3, :), "LineStyle", "--");
  %      plot(t, std3valsfgo, "Color", colour_palette(3, :), "LineStyle","--");
        plot(t, normstatefgo, "Color", colour_palette(3, :));


        ylabel(ylabels(substatenum), "Interpreter", "latex", "FontSize", 14);
        %legend(legends(substatenum, axis_mask(substatenum, :) == 1), 'Location', 'best');
        legend(legends, 'Location', 'best', "NumColumns",3);

        subplotnum = subplotnum + 1;
    end

    % Reset for next substate#
    substateelementcounter = 1;
    colour_counter = 1;
end

xlabel("Time [s]");
sgtitle(plot_title);


end 