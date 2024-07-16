IN_USERNAME=$1
IN_SSI_PATH=$2
IN_USERNAME="${IN_USERNAME:=pha}"
IN_SSI_PATH="${IN_SSI_PATH:=/home/${IN_USERNAME}/docker_share}"

# Setup Display
echo "# Setup" >> /home/${IN_USERNAME}/.bashrc
echo "export USER=${IN_USERNAME}" >> /home/${IN_USERNAME}/.bashrc
echo "source ${IN_SSI_PATH}/scripts/setup/term_disp.sh" >> /home/${IN_USERNAME}/.bashrc

# Setup Source Paths
echo "source ${IN_SSI_PATH}/scripts/setup/export_docker_paths.sh" >> /home/${IN_USERNAME}/.bashrc

# Install Jekyll
source ${IN_SSI_PATH}/scripts/install/install_jekyll.sh