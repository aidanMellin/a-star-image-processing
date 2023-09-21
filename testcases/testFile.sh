#!/bin/bash
#run using
#not sure if this will help anyone but me
#bash testFile.sh <java/python> <program name>

testPath="../testcases"
ter="terrain.png"
el="mpp.txt"
route="path.txt"
PROG="$1 $2"



#routesArr=( )
routesArr=("brown"  "red"  "white"  )

testArr=("distanceCalcX" "distanceCalcY" "doubleback" "serpentine" "serpentineWater" "stripWater" "stripElevation" "elevation")
#testArr=( "stripElevation")
#testArr=("distanceCalcX" "distanceCalcY" "elevation")
#testArr=("winter")

declare -a arr=()

i="normal"

for route in "${routesArr[@]}"; do
		arr+=("$PROG $testPath/$i/$ter $testPath/$i/$el $testPath/$i/$route.txt $i$route.png")
done

route="path"
for i in "${testArr[@]}"; do
	arr+=("$PROG $testPath/$i/$ter $testPath/$i/$el $testPath/$i/$route.txt $i$route.png")
done



for i in "${arr[@]}";
do
	echo "$(tput setaf 1) $i $(tput sgr 0)"
	time $i;
	#read -p "Press enter to continue"
done
#echo 

