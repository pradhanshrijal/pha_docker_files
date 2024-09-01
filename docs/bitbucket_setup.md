# Bitbucket Setup

This page provides an alternative usage for the project with the secondary resources on [Bitbucket](https://bitbucket.org).

Setup the recommended folder structure:
```bash
cd /home/${USER}
mkdir schreibtisch
cd schreibtisch
git clone https://bitbucket.org/pradhanshrijal/pha_docker_files --recursive
cd pha_docker_files
```

This can also be done with a script:
```bash
wget https://bitbucket.org/pradhanshrijal/pha_docker_files/raw/9d9cf29c1320b39af04617f11c3314f3e573eadd/docker_share/scripts/setup/setup_ssi.sh
source setup_ssi.sh /home/${USER}/schreibtisch https://bitbucket.org/pradhanshrijal/pha_docker_files
```