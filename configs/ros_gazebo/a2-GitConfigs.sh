#!/bin/sh

echo "####################################################################################################"
echo "##### Setting up git config"
echo "####################################################################################################"

# git config --global user.name "Your Name"
# git config --global user.email you@yourdomain.com
git config --global branch.autosetuprebase always
git config --global pull.rebase true
git config --global rebase.autostash true