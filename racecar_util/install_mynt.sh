#!/usr/bin/env bash
git clone https://github.com/slightech/MYNT-EYE-S-SDK.git
cd MYNT-EYE-S-SDK
make init
make install
sudo usermod -aG video $(whoami)
