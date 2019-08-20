#!/usr/bin/env bash
git clone https://github.com/msteinbeck/tinyspline.git
cd tinyspline
mkdir build && cd build
cmake ..
sudo cmake --build . --target install
