# Gnuplot script for density guided compression

reset
set terminal postscript eps enhanced color 14 size 17cm, 9cm
set output "spectrum.eps"

set multiplot layout 1,2 title "HACC power spectrum - t = 499, n = 1.07 billion particles, 32 GB per field"

array colors[7];
colors[1] = "#FF00FF"
colors[2] = "#006400"
colors[3] = "#CB0707"
colors[4] = "#0000FF"
colors[5] = "#800080"
colors[6] = "#FF00FF"
colors[7] = "#A0522D"

array titles[5];
titles[1] = "ratio=5.0, k = 4096"
titles[2] = "ratio=7.5, k = {/Symbol r}_{max} = 31225"
titles[3] = "ratio=5.8, k = 2n\\^{2/5} = 3565, adaptive"
titles[4] = "ratio=9.8, k = {/Symbol r}_{max}, b_{min} = 18"
titles[5] = "ratio=8.3, k = 2{/Symbol r}_{max}"


# ---------------
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

plot 'data/pk-orig.dat' using 1:2:3 title "original" w errorbars lc rgb "#000000",\
     'data/pk-decomp_1.dat' using 1:2:3 title titles[1] w errorbars lc rgb colors[1],\
     'data/pk-decomp_6.dat' using 1:2:3 title titles[2] w errorbars lc rgb colors[2],\
     'data/pk-decomp_adap.dat' using 1:2:3 title titles[3] w errorbars lc rgb colors[3],\
     'data/pk-decomp_9.dat' using 1:2:3 title titles[5] w errorbars lc rgb colors[5],\
     'data/pk-decomp_8.dat' using 1:2:3 title titles[4] w errorbars lc rgb colors[4]


# ---------------
reset
set title 'discrepancy'
set xlabel "wavenumber k\n"
set size ratio 0.9
set ylabel "ratio"
set key Left reverse below maxcols 1

set grid
unset logscale

set xrange [0:10]
set format y "%.2f"

#set logscale x 10
# set yrange [0.99:1.001]
threshold(x) = 1.01

plot threshold(x) title "threshold" w lines lc rgb "#000000",\
     'data/ratio.txt' using 1:( $3/$2) title titles[1] w lines lc rgb colors[1],\
     'data/ratio.txt' using 1:( $8/$2) title titles[2] w lines lc rgb colors[2],\
     'data/ratio.txt' using 1:( $9/$2) title titles[3] w lines lc rgb colors[3],\
     'data/ratio.txt' using 1:($11/$2) title titles[5] w lines lc rgb colors[5],\
     'data/ratio.txt' using 1:($10/$2) title titles[4] w lines lc rgb colors[4]


unset multiplot