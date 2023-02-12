#!/bin/bash

# Change Terminal Display
if [ "$color_prompt" = yes ]; then
    PS1='[${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u\[\033[00m\]:\[\033[01;34m\]\W\[\033[00m\]]\$ '
else
    PS1='[${debian_chroot:+($debian_chroot)}\u:\W]\$ '
fi
