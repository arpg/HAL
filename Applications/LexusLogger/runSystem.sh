cd ~/Code/velodyne_activation/
./velodyne_on

echo "Velodyne should be on"

cd ~/Code/Builds/CoreDev/HAL/Applications/LexusLogger/
sudo ./LexusLogger -imu microstrain:[gps=1]///dev/ttyACM0 -posys microstrain:// -lidar velodyne:// -cam dc1394:[id=0]// -cam dc1394:[id=1]// -imu pcan://

cd ~/Code/velodyne_activation/
./velodyne_off
