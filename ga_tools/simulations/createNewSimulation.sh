#!/bin/bash

if test -z "$1" -o -z "$2"; then
    echo "Usage: $0 template_dir newsimname"
    exit;
fi

cp -r "$1" "$2";
rm -rf "$2"/CVS;
make -s -C "$2" clean;
echo "Created a new subdirectory $2";
if grep "INSTALL_TYPE.*user" ../../Makefile.conf; then 
    mv $2/Makefile $2/Makefile.devel
    mv $2/Makefile.user $2/Makefile
    echo "Activated the user Makefile"
else
    if [ -e "$1"/CVS ]; then
        echo -e "Please call\n cvs add $2\n cvs add $2/*"
        echo "in order to add the directory to the cvs repository.";
        echo "Please add $2 to the SIMULATIONS variable in the Makefile.conf placed in this directory.";
    fi
fi
echo "    Have Fun!";
