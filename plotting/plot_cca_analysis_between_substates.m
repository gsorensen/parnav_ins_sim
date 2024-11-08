function f=plot_cca_analysis_between_substates(fignum, X, Y, xname, yname, xlabels, ylabels)
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

    colororder(colour_palette);

    Nn = size(X, 1); 
    Mm = size(Y, 1);

    [~, ~, ~, U, V] = canoncorr(X', Y');


    f=figure(fignum); clf;
    tiled = tiledlayout(Nn, Mm);
    title(tiled, sprintf("MEKF Canonical Correlation Analysis between %s and %s", xname, yname));
    xlabel(tiled, xname);
    ylabel(tiled, yname);
    tiled.TileSpacing = 'compact';

    for n = 1 : Nn
        for m = 1 : Mm
            nexttile; hold on;
            plot(U(:,n), V(:,m), '.',"Color",colour_palette(3, :));
            b = V(:,m) \ U(:,n);
            plot(U(:,n), b * U(:,n), '--', "Color",colour_palette(4, :));
            xlabel(xlabels(n));
            ylabel(ylabels(m));
        end
    end
end

