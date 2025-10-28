for file in *.png; do convert "$file" -format jpg -modulate 250 -resize 64x64\> "${file%.png}.jpg"; done

