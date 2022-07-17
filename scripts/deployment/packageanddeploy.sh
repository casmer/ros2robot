#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd ${SCRIPT_DIR}/../../../..
pwd

host=$1

if [ -z "$host" ] ; then
  echo "please provide host name"
  exit 1
else
  echo "deploying to host $1"
  host=$1
fi


./src/ros2robot/scripts/deployment/generate_drive_module_package.sh
scp install_aarch64.tar.gz root@$host:
ssh root@$host 'mkdir deploy; cd deploy; tar -xzvf ../install_aarch64.tar.gz; ./install_package.sh'
