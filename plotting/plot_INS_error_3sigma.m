function f = plot_INS_error_3sigma(fig_num, plot_title, t, xtrue, xest, P, plot_mask, axis_mask, filename_to_save_figure_as)

colour_palette = [
    hex2rgb("0072b2"); % blue
    hex2rgb("e69f00"); % orange
    hex2rgb("009e73"); % blue green
    hex2rgb("cc79a7"); % pale violet 
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

legends = [
    {'N','E','D'};
    {'x','y','z'};
    {'Roll','Pitch','Yaw'};
    {'x','y','z'};
    {'Roll rate','Pitch rate','Yaw rate'};
];

% No mask provided, plot all
if nargin == 5
    plot_mask = [1 1 1 1 1];
end

if nargin < 6
    axis_mask = [
        1 1 1;
        1 1 1;
        1 1 1;
        1 1 1;
        1 1 1;
    ];
end

% How many subplots
D = 5;
deg_mask = [0 0 1 0 1];

dx = xtrue - xest;

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
        for id = start_id : end_id
            if deg_mask(substatenum) == 1
                dx(id, :) = ssa(dx(id, :)) .* (180/pi);
            end
            
            if axis_mask(substatenum, substateelementcounter) == 1
                plot(t, dx(id, :), "Color", colour_palette(colour_counter, :));
            end

            substateelementcounter = substateelementcounter + 1;
            colour_counter = colour_counter + 1;
        end

        colour_counter = 1;
        substateelementcounter = 1;


        for id = start_id : end_id 
            Pi = reshape(P(id,id,:), [1, N]);
            bounds = 3 * Pi.^(1/2);

            if deg_mask(substatenum) == 1
                bounds = bounds .* (180/pi);
            end
         
            if axis_mask(substatenum, substateelementcounter) == 1
                plot(t,-bounds, "--","Color", colour_palette(colour_counter, :));
                plot(t, bounds,  "--","Color", colour_palette(colour_counter, :));
            end
            substateelementcounter = substateelementcounter + 1;
            colour_counter = colour_counter + 1;
        end
        
        ylabel(ylabels(substatenum), "Interpreter", "latex", "FontSize", 14);
        legend(legends(substatenum, axis_mask(substatenum, :) == 1), 'Location', 'best');

        subplotnum = subplotnum + 1;
    end

    % Reset for next substate#
    substateelementcounter = 1;
    colour_counter = 1;
end

xlabel("Time [s]");
sgtitle(plot_title);

if nargin == 7
    name = sprintf("%s.pdf", filename_to_save_figure_as);
    exportgraphics(f, name,'ContentType','vector');
end

end 