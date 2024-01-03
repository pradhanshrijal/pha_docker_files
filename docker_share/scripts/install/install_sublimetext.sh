#!/bin/bash

sudo apt update
sudo apt install apt-transport-https ca-certificates curl \
                        software-properties-common -y
curl -fsSL https://download.sublimetext.com/sublimehq-pub.gpg | \
        sudo apt-key add -
sudo add-apt-repository "deb https://download.sublimetext.com/ apt/stable/"
sudo apt update
sudo apt install sublime-text -y
sudo cp /etc/apt/trusted.gpg /etc/apt/trusted.gpg.d