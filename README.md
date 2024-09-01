# pha_docker_files

This package consists of docker files to automate container creation and runs.

# Folder Structure Setup
See [Bitbucket based setup](docs/bitbucket_setup.md) for alternative usage.

Setup the recommended folder structure:
```bash
cd /home/${USER}
mkdir schreibtisch
cd schreibtisch
git clone https://github.com/pradhanshrijal/pha_docker_files --recursive
cd pha_docker_files
```

This can also be done with a script:
```bash
wget https://raw.githubusercontent.com/pradhanshrijal/pha_docker_files/master/docker_share/scripts/setup/setup_ssi.sh
source setup_ssi.sh
```

# Usage
Easy sample run with docker compose:
```bash
./docker_scripts/run-compose.sh
```

For full usage see [Scripts-Usage](https://github.com/pradhanshrijal/pha_docker_files/wiki/Scripts-Usage). See [PHA-Images](https://github.com/pradhanshrijal/pha_docker_files/wiki/PHA-Images) for a list of available images.

# License
`pha_docker_files` is released under the [MIT](LICENSE) License.
