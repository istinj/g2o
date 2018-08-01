reset
# set term pngcairo dashed
# set out 'plots.png'

# set terminal epslatex color
# set out 'plots.tex'

#set terminal postscript eps enhanced color font 'Helvetica,10'
set terminal postscript eps enhanced color
set output 'plots.eps'


set logscale y 10
set grid x y

set xlabel "Iteration"
set ylabel "chi2" offset 1,0
set xrange [0:100]

linethickness = 1.5
# linetype 2 -> dashed

set key out vert center right

set style line 1 linetype 1 linecolor rgb "#7F2723" linewidth linethickness #red

set style line 2 linetype 1 linecolor rgb "#CBB200" linewidth linethickness #yellow_0
set style line 3 linetype 1 linecolor rgb "#7F6F00" linewidth linethickness #yellow_1

set style line 4 linetype 1 linecolor rgb "#00ACCB" linewidth linethickness #blue_0
set style line 5 linetype 1 linecolor rgb "#006B7F" linewidth linethickness #blue_1
set style line 6 linetype 1 linecolor rgb "#00353F" linewidth linethickness #blue_2

set style line 7 linetype 1 linecolor rgb "#00CB64" linewidth linethickness #green_0
set style line 8 linetype 1 linecolor rgb "#007F3F" linewidth linethickness #green_1
set style line 9 linetype 1 linecolor rgb "#003F1F" linewidth linethickness #green_2

set title 'Factor Comparisons'
plot '../graph_0_all.stats' using 2:8 with lines linestyle 1 title 'all', \
'../graph_0_homogeneous.stats' using 2:8 with lines linestyle 2 title 'hom', \
'../graph_0_inhomogenous.stats' using 2:8 with lines linestyle 3 title 'non-hom', \
'../graph_0_ln_lnpt.stats' using 2:8 with lines linestyle 4 title 'ln+lnpt', \
'../graph_0_pl_plln.stats' using 2:8 with lines linestyle 5 title 'pl+plln', \
'../graph_0_pl_plln_plpt.stats' using 2:8 with lines linestyle 6 title 'pl+plln+plpt', \
'../graph_0_pt.stats' using 2:8 with lines linestyle 7 title 'pt', \
'../graph_0_ln.stats' using 2:8 with lines linestyle 8 title 'ln', \
'../graph_0_pl.stats' using 2:8 with lines linestyle 9 title 'pl'

set term x11
