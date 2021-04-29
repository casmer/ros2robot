#!/bin/sh

platform=armhf
deploy_dir=deploy
system_dest_dir=opt
dest_root=target_system
app_dir=cotsbot
ros_package_dir=ros_packages

ros_package_source_dir=install_$platform
target_system_source=src/ros2robot/scripts/target_system

deploy_app_dir=$deploy_dir/$dest_root/$system_dest_dir/$app_dir
deploy_root=$deploy_dir/$dest_root


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

cd $deploy_root

find ./ | grep -i ros_packages
find ./ | grep -v ros_packages


