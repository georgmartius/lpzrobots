#!/bin/bash

if test -z "$1"; then
    echo "usage : $0 Logfile numberoflinesperfile";
    exit 1;
fi

FILE=$1;
TOTAL=`cat "$FILE" | wc -l`;
echo "Total number of lines: $TOTAL";

if test -z "$2"; then
    echo "usage : $0 Logfile numberoflinesperfile";
    exit 1;
fi

NUM=$(($2));
grep "#"  "$FILE" > "$FILE.head"
grep "#C" "$FILE.head" > "$FILE.fieldnames"
split --lines="$NUM" -d "$FILE" "${FILE%.log}_";

FIRST="";
for F in "${FILE%.log}_"*; do
    if test -z $FIRST; then
	FIRST="YES";
	mv "$F" "$F.log"
    else
	cat "$FILE.fieldnames" "$F" > "$F.log"
	rm "$F";
    fi
done

