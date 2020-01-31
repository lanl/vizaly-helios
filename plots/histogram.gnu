# Gnuplot script for particle bucketing histogram

reset
set terminal postscript eps enhanced color 14 size 16cm, 10cm
set output "histogram.eps"
set multiplot layout 1,2 title 'particles and density distribution'
set datafile separator whitespace

# ---------------
array colors[7];
colors[1] = "#FF00FF"
colors[2] = "#006400"
colors[3] = "#CB0707"
colors[5] = "#A0522D"
colors[4] = "#0000FF"
colors[6] = "#800080"
colors[7] = "#A0522D"

array titles[4];
titles[1] = "k = {/Symbol r}_{max} = 31225, fixed"
titles[2] = "k = 20000, fixed"
titles[3] = "k = 4096, fixed"
titles[4] = "k = 3565, adaptive"

# ---------------
set title 'density'
set xlabel "{/Symbol r}"
set size ratio 0.95
set ylabel "cells\n"
set format y "10^{%2T}"
set key Left reverse below maxcols 1
set grid mytics xtics
set logscale y 10

plot 'data/density_distrib_1.dat' using 1:2 title titles[3] lc rgb colors[3] with boxes,\
     'data/density_distrib_2.dat' using 1:2 title titles[2] lc rgb colors[2] with boxes,\
     'data/density_distrib_7.dat' using 1:2 title titles[1] lc rgb colors[1] with boxes,\
     'data/density_distrib_adapt.dat' using 1:2 title titles[4] lc rgb colors[4] with boxes


# ---------------
reset
set title 'particles'
set xlabel "bins"
set size ratio 0.95
set ylabel "particles\n"
set format y "10^{%2T}"
set key Left reverse below maxcols 1
set grid mytics xtics
set logscale y 10

plot 'data/particle_distrib_1.dat' using 1:2 title titles[3] lc rgb colors[3] with boxes,\
     'data/particle_distrib_2.dat' using 1:2 title titles[2] lc rgb colors[2] with boxes,\
     'data/particle_distrib_7.dat' using 1:2 title titles[1] lc rgb colors[1] with boxes,\
     'data/particle_distrib_adapt.dat' using 1:2 title titles[4] lc rgb colors[4] with boxes

# ---------------
unset multiplot