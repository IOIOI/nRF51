#!/bin/bash

FILES=$(cat Makefile_Original | grep '\\' | grep abspath | awk '{print $2}' | awk -F ')' '{print $1}');
TARGET="../../../headers/";
TOCOPY=""


if [ -d $TARGET ]
then
	rm -r $TARGET
fi

for i in $FILES
do

# Copy c files
	SIZE=$(echo $i | awk '{n=split($0, array, "/")} END{print n }')

	for((var=1; var<$SIZE; var++))
	do
		PART=$(echo $i | awk -v x=$var -F "/" '{print $x}')

		if [ $PART = ".." ]
		then
			continue
		fi

		TARGET="${TARGET}${PART}/"
	done

	if [ ! -d $TARGET ]
	then
		mkdir -p $TARGET
	fi

	cp -v $i $TARGET

# Find missing headers

	HEADERS=$(cat $i | grep "#include" | grep "\"" | awk -F '"' '{print $2}')

	for j in $HEADERS
	do
		TMP=$(find ../../../../../../ -type f -name $j | grep -v examples | grep -v ble_peripheral)

	        SIZE=$(echo $TMP | awk '{n=split($0, array, " ")} END{print n }')

		if [ $SIZE -gt 1 ]
		then
 		       	for((var=1; var<$SIZE; var++))
	        	do
                		PART=$(echo $TMP | awk -v x=$var '{print $x}')
				TMPPART=$(echo $PART | grep s130)
				TOCOPY="$TOCOPY $TMPPART"

	        	done
		else
			TOCOPY="$TOCOPY $TMP"
		fi
	done

	TOCOPY=$(echo $TOCOPY | sed -e "s/ /\n/g" | sort -u)

	for j in $TOCOPY
	do
		TARGET="../../../headers/"

	        SIZE=$(echo $j | awk '{n=split($0, array, "/")} END{print n }')

        	for((var=1; var<$SIZE; var++))
	        do
        	        PART=$(echo $j | awk -v x=$var -F "/" '{print $x}')

	                if [ $PART = ".." ]
	                then
                        	continue
                	fi

        	        TARGET="${TARGET}${PART}/"
	        done

		FILENAME=$(echo $j | awk -v x=$SIZE -F "/" '{print $x}')
		TARGETFILE="${TARGET}${FILENAME}"

		if [ ! -f $TARGETFILE ]
		then
		        if [ ! -d $TARGET ]
	        	then
        		        mkdir -p $TARGET
		        fi

			cp -v $j $TARGET
		fi
	done

	TARGET="../../../headers/"
done

exit

echo ""
echo ""

FILES=$(find $TARGET -regex '.*\.\(c\|cpp\|h\)$' -print)
HEADERS=""

for i in $FILES
do
	TMP=$(cat $i | grep "#include" | grep "\"" | awk -F '"' '{print $2}')
	HEADERS="$HEADERS $TMP"
done

HEADERS=$(echo $HEADERS | sed -e "s/ /\n/g" | sort -u)
TOCOPY=""

for i in $HEADERS
do
	SEARCH=$(find ../../../headers/ -name $i)

	if [ -z "$SEARCH" ]
	then
		TOCOPY="$TOCOPY $i"
	fi
done



