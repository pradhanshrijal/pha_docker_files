#!/bin/bash
sudo apt install ruby -y
sudo apt-get install ruby-dev -y
sudo gem install ruby
sudo gem install bundler
sudo gem install jekyll
sudo gem install jekyll-paginate

echo fs.inotify.max_user_watches=524288 | sudo tee -a /etc/sysctl.conf && sudo sysctl -p