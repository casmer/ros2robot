apt-get update
apt-get install -y software-properties-common

# Install Raspberry Pi library that we have not provided a rosdep rule for

add-apt-repository ppa:ubuntu-pi-flavour-makers/ppa
apt-get update

#this is for ubuntu 18
#apt-get -y install libwiringpi2 libwiringpi2-dev
#this is for ubuntu 20
apt-get -y install libwiringpi2 libwiringpi-dev
#apt-get -y install libwiringpi2 
#find / -name *wiringPi*
#ln -s /usr/lib/arm-linux-gnueabihf/libwiringPi.so.2 /usr/lib/arm-linux-gnueabihf/libwiringPi.so
