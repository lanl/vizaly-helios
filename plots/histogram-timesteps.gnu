# Gnuplot script for particle bucketing histogram

reset
set terminal postscript eps enhanced color 14 size 18cm, 10cm
set output "histogram-timesteps.eps"
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

array titles[5];
titles[1] = "k = {/Symbol r}_{max}, t = 042"
titles[2] = "k = {/Symbol r}_{max}, t = 272"
titles[3] = "k = {/Symbol r}_{max}, t = 499"

# ---------------
set title 'density'
set xlabel "{/Symbol r}"
set size ratio 0.95
set ylabel "cells"
#set format y "10^{%2T}"
set format y "%.0f"
set key Left reverse below maxcols 1
set grid mytics xtics
set logscale y 10


plot 'data/STEP499/density_distrib_7.dat' using 1:2 title titles[3] lc rgb colors[3] with boxes,\
     'data/STEP272/density_distrib.dat' using 1:2 title titles[2] lc rgb colors[2] with boxes,\
     'data/STEP042/density_distrib.dat' using 1:2 title titles[1] lc rgb colors[1] with boxes



# ---------------
reset
set title 'particles'
set xlabel "bins"
set size ratio 0.95
set ylabel "particles"
#set format y "10^{%2T}"
set format y "%.0f"
set key Left reverse below maxcols 1
set grid mytics xtics
set logscale y 10

plot 'data/STEP042/particle_distrib.dat' using 1:2 title titles[3] lc rgb colors[3] with boxes,\
     'data/STEP272/particle_distrib.dat' using 1:2 title titles[2] lc rgb colors[2] with boxes,\
     'data/STEP499/particle_distrib_7.dat' using 1:2 title titles[1] lc rgb colors[1] with boxes
# ---------------
unset multiplot