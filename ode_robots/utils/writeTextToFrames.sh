#!/bin/bash

if [ -z "$1" -o -z "$2" -o -z "$3" ]; then
  cat <<EOI
    Usage: $0 from to drawcommand {furtheroptions}
       from : index to start
       to   : index to end
       drawcommand: imagemagicks draw command passed to -draw eg: "gravity South text 0,80 'My Message'"
       furtheroptions: more cmd line options default: "-pointsize 40 -fill rgb(200,200,200)"
EOI
 exit 1
fi

OPT=${4:- -pointsize 40 -fill rgb(200,200,200)};
echo $OPT
mkdir -p new;
I=$1;
while [ $I -le $2 ]; do
    printf -v F "frame_%06i.jpg" "$I";
    echo "write new $F to new/$F";
    convert $OPT -draw "$3" $F new/$F;
    I=$((I+1))
done
#for F in frame_0010*; do convert -pointsize 40 -fill "rgb(255,222,0)" -draw "text 140,550 'Motor Cross Connection Changed'" $F new/$F; done
