#!/bin/bash

if test -z "$1" -o -z "$2"; then
    echo "Usage: $0 template_dir newsimname"
    exit;
fi

cp -r "$1" "$2";
rm -rf "$2"/CVS;

echo -e "Call \n cvs add $2";