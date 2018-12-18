#!/bin/bash
sudo ./apt_install_basic.sh
sudo ./apt_install_android.sh
sudo ./apt_install_python_utils.sh
./auto_config_bashrc.sh
./auto_config_ccache.sh
./auto_config_vim.sh
./auto_install_ide.sh
