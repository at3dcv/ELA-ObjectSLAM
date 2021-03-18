#!/bin/bash

echo "###################"
echo "######## 3 ########"
echo "###################"

# AC: somehow nginx is not installed properly
sudo apt-get install nginx

mkdir -p /var/run/sshd

chown -R root:root /root
mkdir -p /root/.config/pcmanfm/LXDE/
cp /usr/share/doro-lxde-wallpapers/desktop-items-0.conf /root/.config/pcmanfm/LXDE/

echo "###################"
echo "####### 13 ########"
echo "###################"

if [ -n "$VNC_PASSWORD" ]; then
    echo "###################"
    echo "####### 18 ########"
    echo "###################"
    echo -n "$VNC_PASSWORD" > /.password1
    x11vnc -storepasswd $(cat /.password1) /.password2
    chmod 400 /.password*
    sed -i 's/^command=x11vnc.*/& -rfbauth \/.password2/' /etc/supervisor/conf.d/supervisord.conf
    export VNC_PASSWORD=
fi

echo "###################"
echo "####### 31 ########"
echo "###################"

sudo fuser -k 80/tcp

cd /usr/lib/web && ./run.py > /var/log/web.log 2>&1 &
nginx -c /etc/nginx/nginx.conf
# AC: not using Docker...
# exec /bin/tini -- /usr/bin/supervisord -n
sudo supervisord -n

echo "###################"
echo "####### END #######"
echo "###################"