# vo_backend

## svo的编译
1. 安装sophus
```
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build
cd build
cmake ..
make
sudo make install

git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local/Sophus_not_template ..
make
sudo make install 
```
2. 安装fast库
```
git clone https://github.com/uzh-rpg/fast.git
cd fast
mkdir build
cd build
cmake ..
```
3. 安装vikit
```
git clone https://github.com/uzh-rpg/rpg_vikit.git
```
4. 安装g2o(可选)
```
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build
cmake ..
sudo make install
```

## svo的运行
### 运行数据集
1. 下载数据集
```
rpg.ifi.uzh.ch/datasets/airground_rig_s3_2013-03-18_21-38-48.bag
```
2. 运行
```
roslaunch svo_ros test_rig3.launch
rosrun rviz rviz -d src/rpg_svo/svo_ros/rviz_config.rviz
rosbag play airground_rig_s3_2013-03-18_21-38-48.bag
```
3. 崩溃问题的解决
将march相关的选项关掉
### 运行realsense d435i
1. 安装realsense的驱动，并且启动realsense相机
2. 添加realsense的svo的配置
