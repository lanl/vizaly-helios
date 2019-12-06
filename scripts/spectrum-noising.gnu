# Gnuplot script for entropy analysis

reset
set terminal postscript eps enhanced color 14 size 16cm, 9cm
set output "spectrum.eps"
set multiplot layout 1,2 title 'primordial power spectrum - m000.full.mpicosmo.499 - 1073726359 particles'

# next colors '#FF00FF' and '#A0522D'

# ---------------
set title ''
set size ratio 1
set xlabel "k (h/Mpc)"
set ylabel "P_0(k) (Mpc/h)^3"
#set key right maxcols 2
set key Left reverse below maxcols 2
set grid
set logscale x 10
set logscale y 10
set xrange [0.01:10]

plot 'pk-orig.dat'         using 1:2:3 title 'original'     w errorbars lc rgb '#FF00FF',\
     'pk-noised-0.25.dat'  using 1:2:3 title 'noised: range=[0-0.25], mean=0.125, dev=0.025' w errorbars lc rgb '#CB0707',\
     'pk-noised-0.5-2.dat' using 1:2:3 title 'noised: range=[0-0.50], mean=0.250, dev=0.012' w errorbars lc rgb '#006400',\
     'pk-noised-0.5-1.dat' using 1:2:3 title 'noised: range=[0-0.50], mean=0.250, dev=0.025' w errorbars lc rgb '#0000FF',\
     'pk-noised-0.5.dat'   using 1:2:3 title 'noised: range=[0-0.50], mean=0.250, dev=0.050' w errorbars lc rgb '#800080',\
     'pk-noised-1.dat'     using 1:2:3 title 'noised: range=[0-1.00], mean=0.500, dev=0.100' w errorbars lc rgb '#000000'


# ---------------
reset
set title ''
set xlabel "wavenumber k (h/Mpc)"
set size ratio 0.9
#set ylabel "P_{zip}/P_{raw}"
set ylabel "discrepancy"
#set key right maxcols 2
set key Left reverse below maxcols 2

set grid
unset logscale

set xrange [0:10]
set yrange [0.5:1.2]

plot 'ratio.dat' using 1:($3/$2) title 'noised: range=[0-0.25], mean=0.125, dev=0.025' w lines lc rgb '#CB0707',\
              '' using 1:($4/$2) title 'noised: range=[0-0.50], mean=0.250, dev=0.012' w lines lc rgb '#006400',\
              '' using 1:($4/$2) title 'noised: range=[0-0.50], mean=0.250, dev=0.025' w lines lc rgb '#0000FF',\
              '' using 1:($5/$2) title 'noised: range=[0-0.50], mean=0.250, dev=0.050' w lines lc rgb '#800080',\
              '' using 1:($6/$2) title 'noised: range=[0-1.00], mean=0.500, dev=0.100' w lines lc rgb '#000000'

unset multiplot
