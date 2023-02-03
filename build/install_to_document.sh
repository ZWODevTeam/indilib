#!/bin/bash
dir=~/project/install_indi/$(basename $(dirname $(realpath $0)))
echo "rm $dir"
sudo rm -r  $dir
echo "install to $dir"
sudo make install DESTDIR=$dir
