#!/bin/bash
cd month
for filename in *.svg; do
	bname=$(basename "$filename" .svg)
	inkscape -z -b '#000000' -h 28 -e $bname.png "$filename"
	magick convert $bname.png BMP3:$bname.bmp
done
cd ..
#cd digit
#for filename in *.svg; do
#	bname=$(basename "$filename" .svg)
#	inkscape -z -b '#000000' -h 40 -e $bname.png "$filename"
#	magick convert $bname.png BMP3:$bname\_40.bmp
#done
#cd ..
