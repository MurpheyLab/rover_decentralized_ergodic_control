# Activate virtual environment
Python Virtualenvs:
- Virtualenvs work differently on Windows that they do on POSIX systems
- Check this guide for an overview of the different commands: https://docs.python.org/3/library/venv.html
<venv>\Scripts\activate.bat

# Touch Screen Interface Setup
## Installation

### Installation using requirements.txt
Run  `python -m pip install -r requirements.txt` to grab the correct version of the python packages needed for running the touchscreen.

### Installation without using requirements.txt
Upgrade pip3 to the latest version

Install Kivy:
Add the Kivy repo: sudo add-apt-repository ppa:kivy-team/kivy
Install Kivy: sudo apt-get install python3-kivy
(you can also install Kivy using pip3. This might be the best way to do it if you plan on running the touchscreen interface within a python virtual environment).

Install scipy:
pip3 install scipy

Install OpenCV:
pip3 install opencv-python

Install roslibpy:
pip3 install roslibpy

## Copyright and License
The implementations of SwarmInterface contained herein are copyright (C) 2021 - 2022 by Joel Meyer and Allison Pinosky and are distributed under the terms of the GNU General Public License (GPL) version 3 (or later). Please see the LICENSE for more information.

Contact: joelmeyer@u.northwestern.edu

Lab Info: Todd D. Murphey https://murpheylab.github.io/ Northwestern University
