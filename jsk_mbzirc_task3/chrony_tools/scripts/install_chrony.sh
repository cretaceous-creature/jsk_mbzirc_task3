#!/bin/sh
# a script to setup chrony
# set -e
cwd=`dirname "${0}"`
expr "${0}" : "/.*" > /dev/null || cwd=`(cd "${cwd}" && pwd)`
echo $cwd

# usage
# ./install_chrony.sh server
# ./install_chrony.sh client ip-of-server
# ./install_chrony.sh client-from-source ip-of-server

if [ "$1" = "server" ]; then
    if [ ! -e /etc/chrony ]; then
        echo "Please install chrony first"
        exit 2
    fi
    echo "install chrony as server"
    sudo cp $cwd/../templates/chrony.server.conf /etc/chrony/chrony.conf
elif [ "$1" = "client" ]; then
    if [ ! -e /etc/chrony ]; then
        echo "Please install chrony first"
        exit 2
    fi
    if [ "$2" != "" ]; then
        SERVER_IP="$2"
        echo "install chrony as client and the server is $SERVER_IP"
        sudo sh -c "cat $cwd/../templates/chrony.client.conf | sed \"s/\\\$SERVER_IP/$SERVER_IP/g\" > /etc/chrony/chrony.conf"
    else
        echo "Please specify the IP address of the server"
        exit 3
    fi
elif [ "$1" = "client-from-source" ]; then
    if [ "$2" != "" ]; then
        SERVER_IP="$2"
        echo "install chrony from source as client and the server is $SERVER_IP"
        cd /tmp
        rm -rf chrony
        git clone git://git.tuxfamily.org/gitroot/chrony/chrony.git
        cd chrony
        git checkout -b 1.24 1.24
        ./configure
        make
        sudo mkdir /etc/chrony
        touch chrony.texi chrony.txt
        sudo make install
        sudo sh -c "cat $cwd/../templates/chrony.client.conf | sed \"s/\\\$SERVER_IP/$SERVER_IP/g\" > /etc/chrony.conf"
        # installing chrony to update-rc.d
        sudo update-rc.d -f  ntp remove
        sudo cp $cwd/../templates/chrony_service_for_source /etc/init.d/chrony
        sudo chmod +x /etc/init.d/chrony
        sudo update-rc.d chrony defaults
    else
        echo "Please specify the IP address of the server"
        exit 4
    fi
else
    echo "usage: "
    echo "$0 server"
    echo "$0 client [ip-of-server]"
    echo "$0 client-from-source [ip-of-server]"
    exit 1
fi


echo 'Generating chrony.keys'
sudo sh -c 'echo 1 leus > /etc/chrony/chrony.keys'

echo 'Please run sudo /etc/init.d/chrony restart'
