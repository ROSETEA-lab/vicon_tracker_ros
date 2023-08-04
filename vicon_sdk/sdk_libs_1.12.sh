#!/bin/sh

LIB_FOLDER=/tmp/20230413_145507h/Release/Linux64

# Parsing flags
while getopts :i:u flag
do
    case "${flag}" in
        u)
            echo "Uninstalling Vicon SDK libraries, please wait"
            sudo rm /usr/local/lib/libViconDataStreamSDK_CPP.so
            echo "."
            sudo rm /usr/local/lib/libboost_system-mt-x64.so.1.75.0
            echo "."
            sudo rm /usr/local/lib/libboost_thread-mt-x64.so.1.75.0
            echo "."
            sudo rm /usr/local/lib/libboost_timer-mt-x64.so.1.75.0
            echo "."
            sudo rm /usr/local/lib/libboost_chrono-mt-x64.so.1.75.0
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
            
            echo "Installing Vicon SDK libraries, please wait"
            sudo cp $LIB_FOLDER/libViconDataStreamSDK_CPP.so /usr/local/lib
            echo "."
            sudo cp $LIB_FOLDER/libboost_system-mt-x64.so.1.75.0 /usr/local/lib
            echo "."
            sudo cp $LIB_FOLDER/libboost_thread-mt-x64.so.1.75.0 /usr/local/lib
            echo "."
            sudo cp $LIB_FOLDER/libboost_timer-mt-x64.so.1.75.0 /usr/local/lib
            echo "."
            sudo cp $LIB_FOLDER/libboost_chrono-mt-x64.so.1.75.0 /usr/local/lib
            echo "."
            sudo cp $LIB_FOLDER/DataStreamClient.h /usr/local/include
            echo "."
            sudo cp $LIB_FOLDER/IDataStreamClientBase.h /usr/local/include
            echo "."
            sudo chmod 0755 /usr/local/lib/libViconDataStreamSDK_CPP.so /usr/local/lib/libboost_system-mt-x64.so.1.75.0 /usr/local/lib/libboost_thread-mt-x64.so.1.75.0 /usr/local/lib/libboost_timer-mt-x64.so.1.75.0 /usr/local/lib/libboost_chrono-mt-x64.so.1.75.0 /usr/local/include/DataStreamClient.h /usr/local/include/IDataStreamClientBase.h
            echo "." 
            sudo ldconfig
            echo "."
            sudo rm -r /tmp/20230413_145507h
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


