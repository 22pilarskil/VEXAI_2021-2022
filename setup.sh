
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install python3-pip protobuf-compiler libprotoc-dev

export CUDA_HOME=/usr/local/cuda
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64
export PATH=$PATH:$CUDA_HOME/bin


#Installing PyCUDA
echo "\033[0;96m~/Installing PyCUDA\033[0m"
pip3 install Cython
pip3 install 'pycuda<2021.1'


#Installing torch
echo "\033[0;96m~/Installing torch\033[0m"
wget https://nvidia.box.com/shared/static/p57jwntv436lfrd78inwl7iml6p13fzh.whl -O torch-1.8.0-cp36-cp36m-linux_aarch64.whl
sudo apt-get install libopenblas-base libopenmpi-dev 
pip3 install numpy torch-1.8.0-cp36-cp36m-linux_aarch64.whl


#Installing torchvision
echo "\033[0;96m~/Installing torchvision\033[0m"
sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev
git clone --branch release/0.9 https://github.com/pytorch/vision torchvision
cd torchvision


#Installing TensorRT
echo "\033[0;96m~/Installing TensorRT\033[0m"
current_dir=$(pwd)
if [ ! -f "$current_dir/nv-tensorrt-repo-ubuntu1804-cuda10.2-trt8.0.1.6-ga-20210626_1-1_amd64.deb" ] ; then
    wget "https://www.dropbox.com/s/sdx7c8xtlieor66/nv-tensorrt-repo-ubuntu1804-cuda10.2-trt8.0.1.6-ga-20210626_1-1_amd64.deb?dl=1" -O "$current_dir/nv-tensorrt-repo-ubuntu1804-cuda10.2-trt8.0.1.6-ga-20210626_1-1_amd64.deb"
fi

sudo dpkg --add-architecture amd64

os="ubuntu1804"
tag="cuda10.2-trt8.0.1.6-ga-20210626"
sudo dpkg -i nv-tensorrt-repo-${os}-${tag}_1-1_amd64.deb
sudo apt-key add /var/nv-tensorrt-repo-${os}-${tag}/7fa2af80.pub

sudo apt-get update
sudo apt-get install tensorrt

sudo apt-get install python3-libnvinfer-dev
dpkg -l | grep TensorRT

cd /usr/src/tensorrt/samples/trtexec && sudo make

#Installing pyrealsense2
echo "\033[0;96m~/Installing pyrealsense2\033[0m"
sudo apt-get install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
git clone https://github.com/IntelRealSense/librealsense.git
cd ./librealsense
./scripts/setup_udev_rules.sh
mkdir build && cd build
sudo make uninstall && sudo make clean && sudo make -j4 -DBUILD_WITH_CUDA=true && sudo make install
echo "export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.6/pyrealsense2" >> ~/.bashrc
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.6/pyrealsense2


#Installing other dependencies
echo "\033[0;96m~/Installing other dependencies\033[0m"
pip3 install numpy==1.19.0
sudo apt-get install libatlas-base-dev gfortran
pip3 install scipy==1.5.1
pip3 install pyserial
pip3 install tqdm
pip3 install seaborn
pip3 install gitpython
pip3 install onnx


