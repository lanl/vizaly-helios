# Gnuplot script for density guided compression

reset
set terminal postscript eps enhanced color 14 size 17cm, 9cm
set output "spectrum-steps.eps"

set multiplot layout 1,2 title ""
#"power spectrum, n = 1.07 billion particles, 32 GB per field"

array colors[7];
colors[1] = "#FF00FF"
colors[2] = "#006400"
colors[3] = "#CB0707"
colors[4] = "#0000FF"
colors[5] = "#800080"
colors[6] = "#A0522D"
colors[7] = "#000000"

array titles[4];
titles[1] = "t = 042"
titles[2] = "t = 272"
titles[3] = "t = 499"
titles[4] = "t = 499, bins = 2{/Symbol r}_{max}"
#titles[1] = "ratio=5.0, k = 4096"
#titles[2] = "ratio=7.5, k = {/Symbol r}_{max} = 31225"
#titles[3] = "ratio=5.8, k = 2n\\^{2/5} = 3565, adaptive"
#titles[4] = "ratio=9.8, k = {/Symbol r}_{max}, b_{min} = 18"
#titles[5] = "ratio=8.3, k = 2{/Symbol r}_{max}"
#titles[6] = "ratio=9.5, k = {/Symbol r}_{max}"
#titles[7] = "ratio=3, non-halo extraction"


# ---------------
#reset
set title 'bits'
set xlabel "{/Symbol r} (log scale)"
set ylabel "bits\n"
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
titles[1] = "t = 042, bins = {/Symbol r}_{max}, ratio = 3.23"
titles[2] = "t = 272, bins = {/Symbol r}_{max}, ratio = 7.89"
titles[3] = "t = 499, bins = {/Symbol r}_{max}, ratio = 7.42"

set title 'discrepancy'
set xlabel "k"
set size ratio 0.82
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
     'data/STEP499/pk-ratio.dat' using 1:($3/$2) title titles[3] w lines lc rgb colors[3],\
     'data/STEP499_3/pk-ratio.dat' using 1:($5/$2) title titles[4] w lines lc rgb colors[4]

# 'data/STEP272/pk-ratio.dat' using 1:($3/$2) title titles[2] w lines lc rgb colors[2],\
unset multiplot