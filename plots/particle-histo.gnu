# Gnuplot script for particle bucketing histogram

reset
set terminal postscript eps enhanced color 14 size 10cm, 9cm
set output "binning.eps"

array colors[7];
colors[1] = "#FF00FF"
colors[2] = "#006400"
colors[3] = "#CB0707"
colors[4] = "#0000FF"
colors[5] = "#800080"
colors[6] = "#FF00FF"
colors[7] = "#A0522D"

set datafile separator whitespace
set title 'particle bins distribution'
set xlabel "bucket"
set size ratio 0.95
set ylabel "particles\n"
set format y "10^{%2T}"

set grid mytics xtics
set logscale y 10
# set logscale x 2
set xrange [0:10000]

plot 'data/particle_distrib_2.dat' using 1:2 notitle lc rgb colors[1] with boxes