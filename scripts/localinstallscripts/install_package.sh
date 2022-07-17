#!/bin/sh



system_root=/

package_contents=target_system

cp -vrf $package_contents/* $system_root

systemctl daemon-reload


systemctl enable robotdrivehost.service
systemctl enable robotptzhost.service
systemctl enable robotpowerbutton.service

#systemctl restart robotdrivehost.service
#systemctl restart robotptzhost.service
#systemctl restart robotpowerbutton.service
#find ./ | grep -i ros_packages
#find ./ | grep -v ros_packages


