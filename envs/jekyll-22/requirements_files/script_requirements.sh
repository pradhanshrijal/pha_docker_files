IN_USERNAME=$1
IN_SCRIPTS_PATH=$5
IN_USERNAME="${IN_USERNAME:=pha}"
IN_SCRIPTS_PATH="${IN_SCRIPTS_PATH:=/home/${IN_USERNAME}/docker_share/scripts}"

# Setup Display
echo "# Setup" >> /home/${IN_USERNAME}/.bashrc
echo "export USER=${IN_USERNAME}" >> /home/${IN_USERNAME}/.bashrc
echo "source ${IN_SCRIPTS_PATH}/setup/term_disp.sh" >> /home/${IN_USERNAME}/.bashrc

# Setup Source Paths
echo "source ${IN_SCRIPTS_PATH}/setup/export_docker_paths.sh" >> /home/${IN_USERNAME}/.bashrc

# Install Jekyll
source ${IN_SCRIPTS_PATH}/install/install_jekyll.sh