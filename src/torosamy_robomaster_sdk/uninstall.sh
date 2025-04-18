bin_directory=$(pwd)


sudo rm -rf /opt/torosamy_robomaster_sdk
sudo rm -rf /opt/torosamy_rm_sdk
sudo rm /etc/udev/rules.d/60-u3v-drv.rules
sudo rm /etc/udev/rules.d/88-mvusb.rules
sudo rm /etc/udev/rules.d/99-mvusb.rules
sudo rm /etc/ld.so.conf.d/torosamy-robomaster-core.conf
sudo rm /etc/ld.so.conf.d/torosamy-rm-core.conf
echo 'Uninstalling completed, please restart your computer'
