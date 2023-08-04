#!/bin/sh

LIB_FOLDER=/tmp/20200407_123216h/Linux64/Release

# Parsing flags
while getopts :i:u flag
do
    case "${flag}" in
        u)
            echo "Uninstalling Vicon SDK libraries, please wait"
            sudo rm /usr/local/lib/libViconDataStreamSDK_CPP.so
            echo "."
            sudo rm /usr/local/lib/libboost_system-mt.so.1.58.0
            echo "."
            sudo rm /usr/local/lib/libboost_thread-mt.so.1.58.0
            echo "."
            sudo rm /usr/local/lib/libboost_timer-mt.so.1.58.0
            echo "."
            sudo rm /usr/local/lib/libboost_chrono-mt.so.1.58.0
            echo "."
            sudo rm /usr/local/include/DataStreamClient.h
            echo "."
            sudo rm /usr/local/include/IDataStreamClientBase.h
            echo "."
            sudo ldconfig
            echo "."
            echo "Uninstalllation finished"
            ;;
            
        i)
            # Check the number of input arguments
            if [ $# -lt 2 ]
            then
              echo "Wrong number of arguments."
              echo ""
              echo "Usage:"
              echo "- to install Vicon SDK libraries: sdk_libs -i name_of_sdk_file.zip"
              echo "- to uninstall Vicon SDK libraries: sdk_libs -u"
              exit
            fi

            echo "Unzipping Vicon SDK file " 
            sudo unzip -q $2 -d /tmp
            sudo unzip -q $LIB_FOLDER/ViconDataStreamSDK_1.10.0.123216h.zip -d /tmp/vicon_libs
            
            echo "Installing Vicon SDK libraries, please wait"
            sudo cp /tmp/vicon_libs/libViconDataStreamSDK_CPP.so /usr/local/lib
            echo "."
            sudo cp /tmp/vicon_libs/libboost_system-mt.so.1.58.0 /usr/local/lib
            echo "."
            sudo cp /tmp/vicon_libs/libboost_thread-mt.so.1.58.0 /usr/local/lib
            echo "."
            sudo cp /tmp/vicon_libs/libboost_timer-mt.so.1.58.0 /usr/local/lib
            echo "."
            sudo cp /tmp/vicon_libs/libboost_chrono-mt.so.1.58.0 /usr/local/lib
            echo "."
            sudo cp /tmp/vicon_libs/DataStreamClient.h /usr/local/include
            echo "."
            sudo cp /tmp/vicon_libs/IDataStreamClientBase.h /usr/local/include
            echo "."
            sudo chmod 0755 /usr/local/lib/libViconDataStreamSDK_CPP.so /usr/local/lib/libboost_system-mt.so.1.58.0 /usr/local/lib/libboost_thread-mt.so.1.58.0 /usr/local/lib/libboost_timer-mt.so.1.58.0 /usr/local/lib/libboost_chrono-mt.so.1.58.0
            echo "." 
            sudo ldconfig
            echo "."
            sudo rm -r /tmp/20200407_123216h /tmp/vicon_libs
            echo "."
            echo "Installlation finished"
            ;;
        
        *) 
            echo "Illegal option $1"
            echo ""
            echo "Usage:"
            echo "- to install Vicon SDK libraries: sdk_libs -i name_of_sdk_file.zip"
            echo "- to uninstall Vicon SDK libraries: sdk_libs -u"
            exit
            ;;
    esac
done


