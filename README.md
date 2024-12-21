# Zephyr Template Application

This is a Zephyr Workspace
[T2 topology](https://docs.zephyrproject.org/latest/develop/west/workspaces.html#t2-star-topology-application-is-the-manifest-repository)
that sets up to build the Template Zephyr application for custom esp32 board.

## Setup

Install west: https://docs.zephyrproject.org/latest/develop/west/install.html

Sometimes, need to setup system path to west.exe and install some python modules:

```
pip3 install pyelftools
pip install intelhex

```

```
mkdir <workspace_dir>
cd <workspace_dir>
west init -m https://github.com/andriyhrabchak/app.git
west update
west blobs fetch hal_espressif
```

## Build and flash
Use VSCode Shift+Alt+B to choose action