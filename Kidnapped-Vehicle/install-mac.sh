#! /bin/bash
#brew install openssl libuv cmake zlib
# echo 'export PATH="/usr/local/opt/openssl/bin:$PATH"' >> ~/.bash_profile
export OPENSSL_ROOT_DIR=/usr/local/opt/openssl/
export OPENSSL_INCLUDE_DIR=/usr/local/opt/openssl/include

git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
patch CMakeLists.txt < ../cmakepatch.txt
mkdir build
export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig 
cd build
cmake ..
make 
sudo make install
cd ../..
sudo rm -r uWebSockets
