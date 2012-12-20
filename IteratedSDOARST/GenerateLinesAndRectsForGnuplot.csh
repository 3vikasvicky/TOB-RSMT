#!/bin/csh -f

# Script to parse $lines_dat and $rect_dat file to generate list of
# markers for plotting with gnuplot.  Maximum and minimum of x and y
# cordinates are also stored for gnuplot to make the screen size
# according to it.
# Coded 4 March 2009.  Ashutosh Chakraborty

set lines_dat=$1
set rect_dat=$2
# Read the lines data from $lines_dat file and write its marker.  Remove
# "nohead" if you want, it removes the arrow from the head of the line
awk '{print "set arrow from "$1","$2" to "$3","$4" nohead"}' $lines_dat >!  RunMeInGnuplot.gpl

# Read the rectangle data from $rect_dat file and write its marker
awk '{print "set object "NR" rect from "$1","$2" to "$3","$4" fc ls 2 fs pattern 1 bo -1"}' $rect_dat >> RunMeInGnuplot.gpl
# fs pattern 1 is big net size, fs pattern 2 is smaller; bo -1 is black boundary, 1 is red, 2 is green.
#set object 2 polygon from 20,20 to 20,45 to 40,45 to 40,50 to 65,50 to 65,45 to 80,45 to 80,20 to 20,20 fs pattern 1 bo 2 fc rgbcolor "cyan"

# Put all X cordinates in file xpoints.txt
awk '{print $1}' $lines_dat >! xpoints.txt
awk '{print $3}' $lines_dat >> xpoints.txt
awk '{print $1}' $rect_dat >> xpoints.txt
awk '{print $3}' $rect_dat >> xpoints.txt

# Put all Y codrinates in file ypoints.txt
awk '{print $2}' $lines_dat >! ypoints.txt
awk '{print $4}' $lines_dat >> ypoints.txt
awk '{print $2}' $rect_dat >> ypoints.txt
awk '{print $4}' $rect_dat >> ypoints.txt

# Find min and max of x cordinates
set xmin=`sort -n xpoints.txt | head -1`
set xmax=`sort -nr xpoints.txt | head -1`

# Find min and max of y cordinates
set ymin=`sort -n ypoints.txt | head -1`
set ymax=`sort -nr ypoints.txt | head -1`

# Put the range of x and y in gnuplot command file
echo "set xrange [ $xmin : $xmax ]" >> RunMeInGnuplot.gpl
# echo "set yrange [ $ymin : $ymax ]" >> RunMeInGnuplot.gpl

# Now, print a dummy function.  Since we dont want this to show up (or
# show up as less as possible, we will print it with dots.  The artifact
# is that you would be able to see a line minutely through the screen.
# Ignore this.
echo "plot x with dots" >> RunMeInGnuplot.gpl

# At this stage, your file RunMeInGnuplot.gpl is ready.  On command
# line, run gnuplot and then do "load 'RunMeInGnuplot.gpl'" (without
# double quotes, but with single quote)
