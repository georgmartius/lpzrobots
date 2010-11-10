#!/bin/sh
# Schleifen: while
# mit Erzeugung einer Laufzahl
i=1
while [ $i -le 5 ]
do
  ./start -p $1 $2
#  i=`expr $i + 1`
  echo "!!!!RESTART!!!!"
done