# leap_motion_msgs

## Installation Instructions for Ubuntu 20.04
1. Install Libmesa:
    ```bash
    sudo apt-get install libgl1-mesa-glx
    ```

2. Clone this package into a catkin_ws/src folder:
    ```bash
    git clone https://github.com/iamlab-cmu/leap_motion_msgs
    ```

3. Install the deb file:
    ```bash
    cd leap_motion_msgs/Leap_Developer_Kit
    sudo dpkg -i Leap-2.3.1+33747-x64.deb
    ```

4. Install the service:
    ```bash
    sudo vim /lib/systemd/system/leapd.service
    ```

5. Paste the following lines into the file and then save:
    ```bash
    [Unit]
    Description=LeapMotion Daemon
    After=syslog.target

    [Service]
    Type=simple
    ExecStart=/usr/sbin/leapd

    [Install]
    WantedBy=multi-user.target
    ```

6. Make a symbolic link and then start the service:
    ```bash
    sudo ln -s /lib/systemd/system/leapd.service /etc/systemd/system/leapd.service
    sudo systemctl daemon-reload
    sudo service leapd start
    ```

7. Add the following lines to your ~/.bashrc file:
    ```bash
    export PYTHONPATH=$PYTHONPATH:/path/to/leap_motion_msgs/Leap_Developer_Kit/LeapSDK/python3.8-project
    ```


## Credits

1. [https://forums.leapmotion.com/t/leap-motion-sdk-with-python-3-5-in-linux-tutorial/5249/1](https://forums.leapmotion.com/t/leap-motion-sdk-with-python-3-5-in-linux-tutorial/5249/1)
2. [https://blog.keithkim.com/2020/07/note-leap-motion-on-ubuntu-2004.html](https://blog.keithkim.com/2020/07/note-leap-motion-on-ubuntu-2004.html)