#!/bin/bash
#for f in $(ls -p | grep -v /); do	

echo $0
dirName=$(dirname $0)

for f in $(find $dirName -name '*'); do 
#	name=$(basename $f|grep indi)
#	if [ -z "$name" ];then
#		continue
#	fi	
	if [ ! -f "$f" ];then
			continue
	fi
	
 	execu=$(ls -al $f|grep x)
	if [ -z "$execu" ];then
			continue
	fi

	ret=$(ldd $f|grep libindiAlignmentDriver)
	if [ ! -z "$ret" ];then
		echo $f
		echo $ret
		echo ""
	fi
done

