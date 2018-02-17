#!/bin/bash
cd month
for filename in *.svg; do
	bname=$(basename "$filename" .svg)
	inkscape -z -b '#000000' -h 35 -e $bname.png "$filename"
	magick convert $bname.png BMP3:$bname.bmp
done
