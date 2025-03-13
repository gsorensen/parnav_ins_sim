# PARNAV INS Simulator

## Quick setup

- Setup data and results folders
- Update paths in various scripts and C++ to match desired
- Generate trajectory using ins_mekf/exOtterMSS.m
- Generate simulation data using ins_mekf/generate_simulation_data.m
- Then either build and run ins_fgo with the scripts there, or execute run.m in
  ins_mekf
- Results can be shown with plotting/plot_results.m 

## Matlab dependencies

The Matlab scripts utilise the Marine Systems Simulator by T.I. Fossen and T.
Perez (2004), the repo of which can be found [here](https://github.com/cybergalactic/MSS).
