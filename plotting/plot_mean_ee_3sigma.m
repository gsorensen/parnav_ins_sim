function plot_mean_ee_3sigma(f, plot_mask, t, mean_err_norm, mean_3sigma, counter, legends, plot_err, plot_bounds, M)
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

ylabels = ["Position error norm [m]" , ...
    "Velocity error norm [m/s]", ...
    "Attitude error norm [deg]",...
    'Accelerometer bias error norm [m$/$s$^2$]',...
   "Gyroscope bias error norm [deg$/$s]"
];

D = 5;
numsubplots = sum(plot_mask);
subplotnum = 1;
N = size(t, 2);
deg_mask = [0 0 1 0 1];
numcolumns = max(floor(size(legends, 2)/2),1);
figure(f); hold on;
for substatenum = 1 : D    
    if plot_mask(substatenum) == 1
        subplot(numsubplots,1,subplotnum); hold on; grid on;
        err_norm = mean_err_norm(substatenum,:);
        bound = mean_3sigma(substatenum,:);

        if deg_mask(substatenum) == 1
            err_norm = err_norm .* (180/pi);
            bound = bound .* (180/pi);
        end
       
        if plot_bounds == true
%            area(t, bound, "FaceColor", colour_palette(counter, :), "FaceAlpha", 0.6, "EdgeColor", colour_palette(counter, :), "LineStyle", ":");
            plot(t, bound, "Color", colour_palette(counter, :), "LineStyle", "-.", "LineWidth", 1.5);

        end

        if plot_err == true
            plot(t, err_norm, "Color", colour_palette(counter, :),"LineWidth", 1.5);
        end

        ylabel(ylabels(substatenum), "Interpreter", "latex", "FontSize", 14);

        if subplotnum == 1
        legend(legends, 'Location', 'north', "NumColumns", numcolumns, "Interpreter", "latex", "FontSize",12);
        end
        subplotnum = subplotnum + 1;
    end
    if substatenum == D
    %legend(legends, 'Location', 'best', "NumColumns",3);
    %Lgnd = legend('show');
    %Lgnd.Position(1) = 0.5;
    %Lgnd.Position(2) = -0.3;
    end
    xlim([t(1), t(N)])
    xlabel("Time [s]", "Interpreter", "latex", "FontSize", 14);
    %sgtitle("Left: Average estimation error norm over $M$ runs with $3\sigma$-bounds Right: True yaw angle", "Interpreter", "latex")
end

end

