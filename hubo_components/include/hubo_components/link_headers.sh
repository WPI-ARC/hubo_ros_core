#!/bin/bash

echo "Making symlinks to hubo-ach and hubo-motion-rt headers..."
echo "First, removing any existing headers..."
rm *.h
rm -r hubo
echo "...done!"

for CANDIDATE in `find $HOME/ -name "hubo.h"`
do
    if [[ ! -L $CANDIDATE ]]
    then
        HUBOACHPATH=$(dirname $CANDIDATE)
        echo "Found hubo-ach at: " $HUBOACHPATH
        echo "Symlinking headers..."
        ln -s $HUBOACHPATH/* ./
        echo "...done!"
    fi
done

for CANDIDATE in `find $HOME/ -name "motion-trajectory.h"`
do
    if [[ ! -L $CANDIDATE ]]
    then
        HUBOMOTIONPATH=$(dirname $CANDIDATE)
        echo "Found hubo-motion-rt at: " $HUBOMOTIONPATH
        echo "Symlinking headers..."
        ln -s $HUBOMOTIONPATH/* ./
        echo "...done!"
    fi
done
echo "...all done!"

