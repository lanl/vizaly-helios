# Gnuplot script for entropy analysis

reset
set terminal postscript eps enhanced color 14 size 24cm, 9cm
set output "spectrum.eps"
set multiplot layout 1,3 title 'primordial power spectrum - m000.full.mpicosmo.499 - 1073726359 particles'

# reuse title and colors in all subplots
array titles[5];
titles[1] = "noised: range=[ 0.5, 1.0], mean=0.75, dev=0.10"
titles[2] = "noised: range=[-1.0, 1.0], mean=0.00, dev=0.40"
titles[3] = "noised: range=[-0.5, 0.5], mean=0.00, dev=0.20"
titles[4] = "noised: range=[ 0.0, 1.0], mean=0.50, dev=0.40"
titles[5] = "noised: range=[ 0.0, 1.0], mean=0.50, dev=0.05"

array colors[7];
colors[1] = "#FF00FF"
colors[2] = "#006400"
colors[3] = "#CB0707"
colors[4] = "#0000FF"
colors[5] = "#800080"
colors[6] = "#FF00FF"
colors[7] = "#A0522D"

# ---------------
set title 'power spectrum'
set size ratio 1
set xlabel "k (h/Mpc)"
set ylabel "P_0(k) (Mpc/h)^3"
set key Left reverse below maxcols 2
set grid
set logscale x 10
set logscale y 10
set xrange [0.01:10]

plot 'pk-orig.dat'         using 1:2:3 title "original" w errorbars lc rgb "#000000",\
     'pk-noised-1.dat'     using 1:2:3 title titles[1]  w errorbars lc rgb colors[1],\
     'pk-noised-2.dat'     using 1:2:3 title titles[2]  w errorbars lc rgb colors[2],\
     'pk-noised-3.dat'     using 1:2:3 title titles[3]  w errorbars lc rgb colors[3],\
     'pk-noised-4.dat'     using 1:2:3 title titles[4]  w errorbars lc rgb colors[4],\
     'pk-noised-5.dat'     using 1:2:3 title titles[5]  w errorbars lc rgb colors[5]

# -----------
set title 'noise distribution'
set xlabel "range"
set size ratio 0.95
#set ylabel "P_{zip}/P_{raw}"
set ylabel "frequency"
#set key right maxcols 2
set key Left reverse below maxcols 2

set grid
unset logscale

set xrange [-1:1]
set yrange [0:0.5]
set datafile separator whitespace
plot 'noise_distrib_1.dat' using 1:2 title titles[1] w lines lc rgb colors[1],\
     'noise_distrib_2.dat' using 1:2 title titles[2] w lines lc rgb colors[2],\
     'noise_distrib_3.dat' using 1:2 title titles[3] w lines lc rgb colors[3],\
     'noise_distrib_4.dat' using 1:2 title titles[4] w lines lc rgb colors[4],\
     'noise_distrib_5.dat' using 1:2 title titles[5] w lines lc rgb colors[5]

# ---------------
reset
set title 'discrepancy'
set xlabel "wavenumber k (h/Mpc)"
set size ratio 0.9
set ylabel "P_{noised} / P_{orig}"
set key Left reverse below maxcols 2

set grid
unset logscale

set xrange [0:10]
set yrange [0.5:1.2]

plot 'ratio.txt' using 1:($2/$2) title "original" w lines lc rgb "#000000",\
     'ratio.txt' using 1:($3/$2) title titles[1] w lines lc rgb colors[1],\
     'ratio.txt' using 1:($4/$2) title titles[2] w lines lc rgb colors[2],\
     'ratio.txt' using 1:($5/$2) title titles[3] w lines lc rgb colors[3],\
     'ratio.txt' using 1:($6/$2) title titles[4] w lines lc rgb colors[4],\
     'ratio.txt' using 1:($7/$2) title titles[5] w lines lc rgb colors[5]

unset multiplot
