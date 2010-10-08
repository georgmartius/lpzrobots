#bin/sh!
make clean
# qmake -project # don't do this anymore. We have edited the .pro file by hand
qmake
make -j2