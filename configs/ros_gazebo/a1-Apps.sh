#!/bin/sh

echo "####################################################################################################"
echo "##### Installing packages"
echo "####################################################################################################"


echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Adding repositories"
echo "------------------------------------------------"

# >>>>> Programming
# +++ VCS
sudo add-apt-repository ppa:git-core/ppa -y # latest git
sudo add-apt-repository ppa:eugenesan/ppa -y # SmartGitHg
sudo add-apt-repository ppa:rabbitvcs/ppa -y # git and svn nautilus integration
sudo apt-add-repository ppa:svn/ppa -y # svn

sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y # g++ latest
sudo add-apt-repository ppa:webupd8team/java -y # Java Oracle
sudo add-apt-repository ppa:webupd8team/sublime-text-3 -y # best text editor
sudo add-apt-repository ppa:ubuntu-sdk-team/ppa -y # QT sdk


# >>>>> Multimedia
sudo add-apt-repository ppa:nilarimogard/webupd8 -y # for pulseaudio-equalizer
sudo add-apt-repository ppa:kazam-team/stable-series -y # desktop recording
sudo add-apt-repository ppa:irie/blender -y # 3d modeling
sudo add-apt-repository ppa:shutter/ppa -y # desktop screenshots
sudo sh -c "echo 'deb http://archive.canonical.com/ubuntu/ precise partner' >> /etc/apt/sources.list.d/canonical_partner.list" # Skype
sudo add-apt-repository ppa:videolan/stable-daily -y # latest vlc


# >>>>> Office
sudo apt-get purge libreoffice-core -y # remove old Libre Office
sudo add-apt-repository ppa:libreoffice/libreoffice-4-2 -y # latest Libre Office
sudo add-apt-repository "deb http://archive.canonical.com/ $(lsb_release -sc) partner" -y # Adobe Reader


# >>>>> OS tools
sudo add-apt-repository ppa:tualatrix/ppa -y # Ubuntu Tweak
sudo add-apt-repository ppa:linrunner/tlp -y # power management
sudo add-apt-repository ppa:atareao/atareao -y # my-weather-indicator
sudo add-apt-repository ppa:atareao/nautilus-extensions -y # pdf tools
sudo add-apt-repository ppa:webupd8team/y-ppa-manager -y # graphical ppa management


# >>>>> Dropbox
sudo apt-key adv --keyserver pgp.mit.edu --recv-keys 5044912E
sudo add-apt-repository "deb http://linux.dropbox.com/ubuntu $(lsb_release -sc) main" -y



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Updating packages index"
echo "------------------------------------------------"

sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get dist-upgrade -y



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Installing packages"
echo "------------------------------------------------"

# >>>>> Programming
# +++ gcc | g++
# sudo apt-get install gcc-4.8 g++-4.8 -y
# sudo update-alternatives --remove-all gcc
# sudo update-alternatives --remove-all g++
# sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 50
# sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 50
# sudo update-alternatives --config gcc
# sudo update-alternatives --config g++

sudo apt-get install gdb gdb-doc gdbserver -y
sudo apt-get install cmake cmake-gui -y
# sudo apt-get install ccache -y
# sudo apt-get install colorgcc -y
sudo apt-get install oracle-java7-installer -y
sudo apt-get install sublime-text-installer -y

# +++ VCS
sudo apt-get install git -y
sudo apt-get install smartgithg -y
sudo apt-get install rabbit rabbitvcs-nautilus3 -y
sudo apt-get install subversion -y


# >>>>> Multimedia
sudo apt-get install skype -y
sudo apt-get install flashplugin-installer gsfonts-x11 -y
sudo apt-get install ubuntu-restricted-extras -y
sudo apt-get install libavformat-extra-53 libavcodec-extra-53 -y
sudo apt-get install libdvdread4 -y
sudo /usr/share/doc/libdvdread4/install-css.sh
sudo apt-get install my-weather-indicator -y
sudo apt-get install vlc browser-plugin-vlc -y
sudo apt-get install totem-plugins-extra
sudo apt-get install kazam -y
sudo apt-get install shutter -y
sudo apt-get install gimp gimp-data gimp-data-extras gimp-plugin-registry -y
sudo apt-get install blender -y


# >>>>> Office
sudo apt-get install acroread -y
sudo apt-get install libreoffice -y


# >>>>> OS tools
sudo apt-get install ubuntu-tweak -y
sudo apt-get install indicator-multiload -y
sudo apt-get install indicator-cpufreq -y
sudo apt-get install myunity -y # 12.04
# sudo apt-get install unity-tweak-tool -y # 13.04
 sudo apt-get install assogiate -y # manage file types and extensions
sudo apt-get install tlp tlp-rdw -y
sudo tlp start
sudo apt-get install compiz compiz-plugins compiz-plugins-default compizconfig-settings-manager -y
sudo apt-get install conky-all -y
sudo apt-get install gparted -y
sudo apt-get install filezilla -y
sudo apt-get install y-ppa-manager -y
sudo apt-get install --no-install-recommends gnome-panel -y # usage: gnome-desktop-item-edit --create-new ~/.local/share/applications/
sudo apt-get install systemtap -y
# sudo apt-get install nvidia-cuda-toolkit -y

# +++ sound equalizer
sudo apt-get install pulseaudio-equalizer -y
mkdir -p ~/.pulse/presets

# +++ synaptics touchpad
sudo apt-get install synaptic -y
sudo apt-get install gsynaptics -y
sudo apt-get install kde-config-touchpad -y


# >>>>> nautilus extensions
sudo apt-get install nautilus-pdf-tools -y
sudo apt-get install nautilus-image-tools -y
sudo apt-get install nautilus-image-converter -y
sudo apt-get install nautilus-ideviceinfo -y
sudo apt-get install nautilus-script-manager -y
sudo apt-get install nautilus-script-debug -y
sudo apt-get install nautilus-script-audio-convert -y
sudo apt-get install nautilus-filename-repairer -y
sudo apt-get install nautilus-actions -y --force-yes
sudo apt-get install nautilus-columns -y --force-yes
sudo apt-get install nautilus-compare -y --force-yes
sudo apt-get install nautilus-open-terminal -y --force-yes
sudo apt-get install nautilus-wallpaper -y --force-yes
sudo apt-get install nautilus-wipe -y --force-yes
sudo apt-get install nautilus-gtkhash -y --force-yes
sudo apt-get install seahorse-nautilus -y


# >>>>> Dropbox
sudo apt-get install dropbox -y


# >>>>> Torrents
sudo apt-get install deluge -y
