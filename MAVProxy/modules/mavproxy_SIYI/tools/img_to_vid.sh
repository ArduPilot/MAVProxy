#!/bin/bash

echo "Converting .bin to .jpg"
~/MAVProxy/MAVProxy/modules/mavproxy_SIYI/tools/temp_dir.py 102SIYI_TEM/ -w

echo "Converting to video. Assuming framerate=5"
ffmpeg -hide_banner -framerate 5 -pattern_type glob -i 'tem_jpg/*.jpg' \
  -c:v libx264 -pix_fmt yuv420p output_no_temp.mp4

echo "Adding filenames to .jpgs"
mkdir -p tem_jpg_annotated
for f in tem_jpg/*.jpg; do
    f2=${f#*/}
    if [ ! -f "tem_jpg_annotated/$f2" ]; then  
        convert "$f" \
            -pointsize 24 \
            -fill white \
            -undercolor '#00000080' \
            -gravity North \
            -annotate +0+10 "${f2%.*}" \
            "tem_jpg_annotated/$f2"
    fi
done

echo "Checking for missed files"
ls tem_jpg > tem_jpg.txt
ls tem_jpg_annotated > tem_jpg_annotated.txt
git diff --no-index tem_jpg.txt tem_jpg_annotated.txt
rm tem_jpg.txt tem_jpg_annotated.txt

echo "Converting to video. Assuming framerate=5"
ffmpeg -hide_banner -framerate 5 -pattern_type glob -i 'tem_jpg_annotated/*.jpg' \
  -c:v libx264 -pix_fmt yuv420p output.mp4

