function k=plot_cca_for_all_substate_pairs(fignum, x)
    substate_names = ["Position" "Velocity" "Atittude"...
        "Accelerometer bias" "Gyroscope bias"];
    substate_labels = [
        "N [m]" "E [m]" "D [m]";
        "x [m/s]" "y [m/s]" "z [m/s]";
        "Roll [rad]" "Pitch [rad]" "Yaw [rad]";
        "x [m/s^2]" "y [m/s^2]" "z [m/s^2]";
        "Roll rate [rad]" "Pitch rate [rad]" "Yaw rate [rad]";
    ];
      
    figcount = 0;
    k = repmat(figure, 1, 10);

    for uidx = 1 : 5
        for vidx = uidx : 5
            if uidx == vidx
                continue
            end
    
            usidx = 3 * uidx - 2;
            ueidx = 3 * uidx;
            vsidx = 3 * vidx - 2;
            veidx = 3 * vidx;       
            fprintf("Plotting CCA between %s and %s in figure %d\n", ...
                substate_names(uidx),substate_names(vidx), ...
                fignum + figcount);
            f = plot_cca_analysis_between_substates(fignum + figcount, ...
                                                x(usidx:ueidx, :), ...
                                                x(vsidx:veidx,:), ...
                                                substate_names(uidx), ...
                                                substate_names(vidx), ...
                                                substate_labels(uidx,:),... 
                                                substate_labels(vidx,:));
            figcount = figcount + 1;
            k(figcount) = f;
        end
    end
end