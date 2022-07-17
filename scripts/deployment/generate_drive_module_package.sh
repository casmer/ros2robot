#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd ${SCRIPT_DIR}/../../../..

pwd
platform=aarch64
module_name=drive_module_rpi

deploy_dir=deploy_build
system_dest_dir=opt
dest_root=target_system
app_dir=cotsbot
ros_package_dir=ros_packages
localinstallscript=install_package.sh
localinstallscript_dir=src/ros2robot/scripts/localinstallscripts

ros_package_source_dir=install_$platform
target_system_source=src/ros2robot/scripts/target_system/$module_name

deploy_app_dir=$deploy_dir/$dest_root/$system_dest_dir/$app_dir
deploy_root=$deploy_dir/$dest_root
install_package_name=install_$platform

if [ -d $deploy_dir ]; then
  echo "removing ${deploy_dir}"
  rm -rf $deploy_dir
else
	echo "${deploy_dir} does not exist"
fi

echo "creating ${deploy_dir}"
mkdir $deploy_dir
echo "creating ${deploy_app_dir}/${ros_package_dir}"
mkdir -p $deploy_app_dir/$ros_package_dir


echo "copying Base system packages from ${target_system_source}"

cp -r $target_system_source/* $deploy_root

echo "copying packages from ${ros_package_source_dir}"

cp -r $ros_package_source_dir/* $deploy_app_dir/$ros_package_dir

cp $localinstallscript_dir/$localinstallscript $deploy_dir/
chmod u+x $deploy_dir/$localinstallscript
cd $deploy_dir
tar -zcvf ../$install_package_name.tar.gz *  > file_inventory.txt
#cd $deploy_root


#find ./ | grep -i ros_packages
#find ./ | grep -v ros_packages


