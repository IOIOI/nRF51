#!/bin/bash

OUTPUT_FILE="PMD_nrf51822.jlink"

SOFTDEVICE_FOUND="FALSE"
SOFTDEVICE_FILE="NULL"

APPLICATION_FOUND="FALSE"
APPLICATION_FILE="NULL"

# Check input parameters
while [ $# -gt 0 ]
do
        # Check occurence of the softdevice parameter
        if [ $1 = "-s"  ]
        then
                shift
                
                # Check if softdevice file exists
                if [ ! -f $1 ]
                then
                        echo "Softdevice not found"
                        exit 1
                else
                        SOFTDEVICE_FOUND="TRUE"
                        SOFTDEVICE_FILE=$1
                fi
        fi

        # Check occurence of the application parameter
        if [ $1 = "-a"  ]
        then
                shift
                
                # Check if application file exists
                if [ ! -f $1 ]
                then
                        echo "Application not found"
                        exit 1
                else
                        APPLICATION_FOUND="TRUE"
                        APPLICATION_FILE=$1
                fi
        fi

        shift
done
# End of parameter checking

# Check parameter usage
if ( [ $SOFTDEVICE_FOUND = "FALSE"  ] && [ $APPLICATION_FOUND = "FALSE" ] )
then
        echo "Wrong / missing input parameter!"
        echo "Usage: flash.sh [-s PATH_TO_SOFTDEVICE.hex] [-a PATH_TO_APPLICATION.hex]"
        exit 1
fi

# Prepare JLink script
echo "si SWD" > $OUTPUT_FILE                    # Set SWD as interface
echo "speed 4000" >> $OUTPUT_FILE               # Set speed (default)
echo "device NRF51822_XXAA" >> $OUTPUT_FILE     # Set device
echo "connect" >> $OUTPUT_FILE                  # Establish a connection to the device

if ( [ $SOFTDEVICE_FOUND = "TRUE"  ] && [ $APPLICATION_FOUND = "FALSE" ] )
then
        # Flash only Softdevice
        echo "loadfile $SOFTDEVICE_FILE" >> $OUTPUT_FILE
fi

if ( [ $SOFTDEVICE_FOUND = "FALSE"  ] && [ $APPLICATION_FOUND = "TRUE" ] )
then
        # Flash only Application
        echo "loadfile $APPLICATION_FILE" >> $OUTPUT_FILE
fi

if ( [ $SOFTDEVICE_FOUND = "TRUE"  ] && [ $APPLICATION_FOUND = "TRUE" ] )
then
        # Flash Softdevice and Application
        echo "erase" >> $OUTPUT_FILE            # Erase flash
        echo "loadfile $SOFTDEVICE_FILE" >> $OUTPUT_FILE
        echo "loadfile $APPLICATION_FILE" >> $OUTPUT_FILE
fi

echo "r" >> $OUTPUT_FILE                        # Perform a reset
echo "g" >> $OUTPUT_FILE                        # Start the execution
echo "q" >> $OUTPUT_FILE                        # Exit JLinkExe
# End of preparation of the JLink script

# Start flashing
JLinkExe -commanderscript $OUTPUT_FILE

exit 0
# End of file
