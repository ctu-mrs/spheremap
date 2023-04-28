#!/bin/bash

MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

FILENAME_SUBT=darpa_subt_finals_uav_green.bag

echo "Downloading models from NAS"
wget --no-check-certificate -O $FILENAME_SUBT https://nasmrs.felk.cvut.cz/index.php/s/ZXXFGKROKsDuUpS/download

echo "Done"
