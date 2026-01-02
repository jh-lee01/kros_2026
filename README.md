# kros_2026

## Clone
```
git clone https://github.com/jh-lee01/kros_2026.git
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install
source install/local_setup.bash
```

## Middleware setting
```
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```
Connect companion computer with PX4 ("/ttyUSB0" depends on the hardware ports)
```
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

## Making package with python nodes
```
cd ~/kros_2026/src
ros2 pkg create --build-type ament_python <package_name>
```
