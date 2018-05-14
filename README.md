# Iris+ Indoor localization and navigation

All these steps are performed in Raspberry Pi 2 and Debian

Install `python-pip` and `python3-pip` 

```bash
$ sudo apt-get install python-pip python3-pip
```

 Install `python3-venv` for virtual development environment

```bash
$ sudo apt-get install python3-venv
```

Creat a virtual development environment for `optitrack` or `pozyx` localization hardware

```Bash
$ python3 -m venv dev
$ source dev/bin/actinve
```

Install python dependences

```bash
$ sudo -H pip install pyserial future
```

Install dronekit from source, see 

[DroneKit]: https://github.com/dronekit/dronekit-python

```Bash
$ git clone https://github.com/dronekit/dronekit-python.git
$ sudo apt-get install libxml2 libxml2-dev libxslt1-dev
```

