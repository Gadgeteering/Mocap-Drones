deactivate 2>/dev/null
sudo rm -rf ~/venvs/pseyepy-env
sudo rm -rf ~/pseyepy/build ~/pseyepy/dist ~/pseyepy/*.egg-info

sudo apt update

sudo apt install -y \
  git \
  build-essential \
  python3-dev \
  python3-venv \
  libusb-1.0-0-dev \
  pkg-config \
  cmake
echo "pip setuptools wheel"
sudo apt install python3 python3-pip python3-venv python3-setuptools python3-wheel

echo "pip install numpy scipy cython flask"
sudo apt install python3-numpy python3-scipy cython3 python3-flask

cd ~/pseyepy

python3 -m venv ~/venvs/mocap
source ~/venvs/mocap/bin/activate

echo "apt install libusb-1.0-0-dev build-essential python3-dev"
sudo apt install libusb-1.0-0-dev build-essential python3-dev

cp build/lib.linux-aarch64-cpython-313/pseyepy/cameras*.so pseyepy/

pip install h5py Pillow numpy


sudo pip install . 

python3 -c "import pseyepy; print('pseyepy OK')"