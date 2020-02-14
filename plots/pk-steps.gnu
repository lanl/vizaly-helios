# Gnuplot script for density guided compression

reset
set terminal postscript eps enhanced color 14 size 25cm, 9cm
set output "spectrum-steps.eps"

set multiplot layout 1,3 title "HACC power spectrum - n = 1.07 billion particles, 32 GB per field"

array colors[7];
colors[1] = "#FF00FF"
colors[2] = "#006400"
colors[3] = "#CB0707"
colors[4] = "#0000FF"
colors[5] = "#800080"
colors[6] = "#A0522D"
colors[7] = "#000000"

array titles[3];
#titles[1] = "ratio=5.0, k = 4096"
#titles[2] = "ratio=7.5, k = {/Symbol r}_{max} = 31225"
#titles[3] = "ratio=5.8, k = 2n\\^{2/5} = 3565, adaptive"
#titles[4] = "ratio=9.8, k = {/Symbol r}_{max}, b_{min} = 18"
#titles[5] = "ratio=8.3, k = 2{/Symbol r}_{max}"
#titles[6] = "ratio=9.5, k = {/Symbol r}_{max}"
#titles[7] = "ratio=3, non-halo extraction"

# ---------------
titles[1] = "timestep: 042"
titles[2] = "timestep: 272"
titles[3] = "timestep: 499"

set title 'power spectrum'
set size ratio 0.9
set xlabel "k (h/Mpc)"
set ylabel "P_0(k) (Mpc/h)^3\n"
set key Left reverse below maxcols 1
set grid
set logscale x 10
set logscale y 10
set xrange [0.01:10]
set yrange [1:100000]
#set format y "10^{%2T}"

plot 'data/STEP042/pk-orig.dat' using 1:2:3 title titles[1] w errorbars lc rgb colors[1],\
     'data/STEP272/pk-orig.dat' using 1:2:3 title titles[2] w errorbars lc rgb colors[2],\
     'data/STEP499/pk-orig.dat' using 1:2:3 title titles[3] w errorbars lc rgb colors[3]

# ---------------
reset
set title 'bits distribution'
set xlabel "{/Symbol r} (log scale)"
set ylabel "bits"
set size ratio 0.8
set key Left reverse below maxcols 1
set grid
set yrange [16:32]
set logscale x 10

plot 'data/bits_distrib.dat' using 1:2 title titles[1] w lines lc rgb colors[1],\
     'data/bits_distrib.dat' using 1:2 title titles[2] w lines lc rgb colors[2],\
     'data/bits_distrib.dat' using 1:2 title titles[3] w lines lc rgb colors[3]


# ---------------
reset
titles[1] = "t = 042, bins = {/Symbol r}_{max} = 31225, ratio = 3.23"
titles[2] = "t = 272, bins = {/Symbol r}_{max} = 31225, ratio = 7.89"
titles[3] = "t = 499, bins = {/Symbol r}_{max} = 31225, ratio = 7.42"

set title 'discrepancy'
set xlabel "k"
set size ratio 0.8
set ylabel "P_{zip} / P_{raw}"
set key Left reverse below maxcols 1

set grid
unset logscale

set xrange [0:10]
set format y "%.3f"

#set logscale x 10
# set yrange [0.99:1.001]
threshold(x) = 1.01

plot threshold(x) title "threshold" w lines lc rgb "#000000",\
     'data/STEP042/pk-ratio.dat' using 1:($3/$2) title titles[1] w lines lc rgb colors[1],\
     'data/STEP272/pk-ratio.dat' using 1:($3/$2) title titles[2] w lines lc rgb colors[2],\
     'data/STEP499/pk-ratio.dat' using 1:($3/$2) title titles[3] w lines lc rgb colors[3]

unset multiplot