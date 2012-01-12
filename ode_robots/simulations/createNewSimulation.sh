#!/bin/bash

if test -z "$1" -o -z "$2"; then
    echo "Usage: $0 template_dir newsimname"
    exit;
fi

cp -r "$1" "$2";
make -s -C "$2" clean;
echo "Created a new subdirectory $2";

echo "    Have Fun!";
