reset
# set term pngcairo dashed
# set out 'plot_guess_comparison_high.png'

# set terminal epslatex color
# set out 'plot_guess_comparison_high.tex'

#set terminal postscript eps enhanced color font 'Helvetica,10'
set terminal postscript eps enhanced color
set output 'plot_guess_comparison_high.eps'

dataset = 'torus3d'

set logscale y 10
set grid x y

set xlabel "Iteration"
set ylabel "chi2" offset 1,0
set xrange [0:100]

linethickness = 3
pointsize     = 1
# linetype 2 -> dashed

set key vert top right

set style line 1 linetype 1 linecolor rgb "#7F2723" linewidth linethickness #red

set style line 2 linetype 1 linecolor rgb "#CBB200" linewidth linethickness #yellow_0
set style line 3 linetype 1 linecolor rgb "#7F6F00" linewidth linethickness #yellow_1

set style line 10 linetype 1 linecolor rgb "#00ACCB" linewidth linethickness #blue_0
set style line 11 linetype 1 linecolor rgb "#006B7F" linewidth linethickness #blue_1
set style line 12 linetype 1 linecolor rgb "#00353F" linewidth linethickness #blue_2

set style line 7 linetype 1 linecolor rgb "#00CB64" linewidth linethickness pointtype 13 pointsize pointsize #green_0
set style line 8 linetype 1 linecolor rgb "#007F3F" linewidth linethickness #green_1
set style line 9 linetype 1 linecolor rgb "#003F1F" linewidth linethickness #green_2

set style line 4 linetype 1 linecolor rgb "#FF940A" linewidth linethickness pointtype 2 pointsize pointsize #orange_0
set style line 5 linetype 1 linecolor rgb "#FFB03F" linewidth linethickness #orange_1
set style line 6 linetype 1 linecolor rgb "#FF7425" linewidth linethickness #orange_3

# set style line 10 linecolor rgb "#e31a1c" linewidth linethickness pointtype 0 pointsize pointsize
# set style line 11 linecolor rgb "#fd8d3c" linewidth linethickness pointtype 1 pointsize pointsize
# set style line 12 linecolor rgb "#fecc5c" linewidth linethickness pointtype 2 pointsize pointsize
# set style line 13 linecolor rgb "#1b7837" linewidth linethickness pointtype 8 pointsize pointsize
# set style line 14 linecolor rgb "#7fbf7b" linewidth linethickness pointtype 4 pointsize pointsize
# set style line 15 linecolor rgb "#2166ac" linewidth linethickness pointtype 7 pointsize pointsize
# set style line 16 linecolor rgb "#67a9cf" linewidth linethickness pointtype 13 pointsize pointsize
# set style line 17 linecolor rgb "black" linewidth linethickness pointtype 10 pointsize pointsize

set title 'Guess Comparison Between Chordal and Geodesic - High Noise'
plot '../chordal/output_gn_noguess/'.dataset.'_high.stats' using 1:2 with linespoints linestyle 4 title 'no guess chordal', \
   '../output_gn_noguess/'.dataset.'_high.stats' using 2:8 with linespoints linestyle 7 title 'no guess geodesic', \
      '../chordal/output_gn_spanning/'.dataset.'_high.stats' using 1:2 with lines linestyle 5 title 'spanning chordal', \
         '../chordal/output_gn_odometry/'.dataset.'_high.stats' using 1:2 with lines linestyle 6 title 'odometry chordal', \
            '../output_gn_spanning/'.dataset.'_high.stats' using 2:8 with lines linestyle 8 title 'spanning geodesic', \
               '../output_gn_odometry/'.dataset.'_high.stats' using 2:8 with lines linestyle 9 title 'odometry geodesic'

set term x11

