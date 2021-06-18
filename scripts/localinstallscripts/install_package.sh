#!/bin/sh



system_root=/

package_contents=target_system

cp -vrf $package_contents/* $system_root

systemctl daemon-reload

systemctl restart robotdrivehost.service
systemctl restart robotptzhost.service
#find ./ | grep -i ros_packages
#find ./ | grep -v ros_packages


