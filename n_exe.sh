#!/bin/bash

i=0
while [ $i -lt $1 ]; do
./parking $2 $3 $4
i=$(( i + 1 ))
done

exit 0
