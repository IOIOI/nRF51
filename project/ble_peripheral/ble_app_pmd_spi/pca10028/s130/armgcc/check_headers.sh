#!/bin/bash

TARGET="../../../headers/"
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

TMPTOCOPY=$(echo $TOCOPY | sed -e "s/ /\n/g" | sort -u)
TOCOPY=""

for i in $TMPTOCOPY
do
#	echo $i
	FILE=$(find ../../../../../../ -name $i | grep -v examples)

	if [ -z "$FILE" ]
	then
		continue
	fi

	SIZE=$(echo $FILE | awk '{n=split($0, array, " ")} END{print n }')

	if [ $SIZE -gt 1 ]
	then
                for((var=1; var<$SIZE; var++))
                do
	                PART=$(echo $FILE | awk -v x=$var '{print $x}')
                        TMP=$(echo $PART | grep s130)
#			TOCOPY="$TOCOPY $TMP"
                done
#	else
#		TOCOPY="$TOCOPY $FILE"
	fi
echo $FILE
done

#echo $TOCOPY

#exit
        for j in $TOCOPY
        do
		echo $j
		continue

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

