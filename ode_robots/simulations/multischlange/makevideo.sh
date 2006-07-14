#/bin/bash
NAME=frame_;

echo -e "********************** making ppm files *************************";
#for F in $NAME*.bmp; do bmptoppm $F > ${F%bmp}ppm; done

echo -e "********************** making jpg files *************************";
#for F in $NAME*.ppm; do ppmtojpeg $F > ${F%ppm}jpg; done

rm *.ppm

echo -e "*********************** mjpeg encoding **************************";
jpeg2yuv -f 25 -I p -j frame_%06d.jpg | mpeg2enc -o mpegfile.m1v

rm *.jpg