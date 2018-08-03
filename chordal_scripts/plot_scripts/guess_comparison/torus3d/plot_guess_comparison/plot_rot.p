reset
# set term pngcairo dashed
# set out 'plot_guess_comparison_rot.png'

# set terminal epslatex color
# set out 'plot_guess_comparison_rot.tex'

#set terminal postscript eps enhanced color font 'Helvetica,10'
set terminal postscript eps enhanced color
set output 'plot_guess_comparison_rot.eps'


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

set title 'Guess Comparison Between Chordal and Geodesic - High Rotational Noise'
plot '../chordal/output_gn_noguess/torus3d_rot.stats' using 1:2 with linespoints linestyle 4 title 'no guess chordal', \
'../output_gn_noguess/torus3d_rot.stats' using 2:8 with linespoints linestyle 7 title 'no guess geodesic', \
'../chordal/output_gn_spanning/torus3d_rot.stats' using 1:2 with lines linestyle 5 title 'spanning chordal', \
'../chordal/output_gn_odometry/torus3d_rot.stats' using 1:2 with lines linestyle 6 title 'odometry chordal', \
'../output_gn_spanning/torus3d_rot.stats' using 2:8 with lines linestyle 8 title 'spanning geodesic', \
'../output_gn_odometry/torus3d_rot.stats' using 2:8 with lines linestyle 9 title 'odometry geodesic'

set term x11

