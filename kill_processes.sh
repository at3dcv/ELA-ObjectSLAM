#!/bin/bash

# Kill noVNC ports
netstat -ltn | grep '0\.0\.0\.0\:[0-9][0-9][0-9][0-9]' | awk '{print $4}' | while read -r line ; do sudo kill  -9 $(sudo lsof -t -i:${line: -4}) ; done

# Kill xvfb ports
sudo kill -9 `ps aux | grep -i xvfb | grep root | awk '{print $2}'`

