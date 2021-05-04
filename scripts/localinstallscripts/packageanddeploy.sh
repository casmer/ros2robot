

host=$1

if [ -z "$host" ] ; then
  echo "please provide host name"
  exit 1
else
  echo "using alternate host $1"
  host=$1
fi


./src/ros2robot/scripts/deploy_drive_module.sh
scp install_aarch64.tar.gz root@$host:
ssh root@$host 'mkdir deploy; cd deploy; tar -xzvf ../install_aarch64.tar.gz; ./install_package.sh'
