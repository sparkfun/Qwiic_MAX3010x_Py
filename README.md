Qwiic_MAX3010x_Py
===================

<p align="center">
   <img src="https://cdn.sparkfun.com/assets/custom_pages/2/7/2/qwiic-logo-registered.jpg"  width=200>  
   <img src="https://www.python.org/static/community_logos/python-logo-master-v3-TM.png"  width=240>   
</p>
<p align="center">
	<a href="https://pypi.org/project/sparkfun-qwiic-max3010x/" alt="Package">
		<img src="https://img.shields.io/pypi/pyversions/sparkfun_qwiic_max3010x.svg" /></a>
	<a href="https://github.com/sparkfun/Qwiic_MAX3010x_Py/issues" alt="Issues">
		<img src="https://img.shields.io/github/issues/sparkfun/Qwiic_MAX3010x_Py.svg" /></a>
	<a href="https://qwiic-max3010x-py.readthedocs.io/en/latest/index.html" alt="Documentation">
		<img src="https://readthedocs.org/projects/qwiic-max3010x-py/badge/?version=latest&style=flat" /></a>
	<a href="https://github.com/sparkfun/Qwiic_MAX3010x_Py/blob/master/LICENSE" alt="License">
		<img src="https://img.shields.io/badge/license-MIT-blue.svg" /></a>
	<a href="https://twitter.com/intent/follow?screen_name=sparkfun">
        	<img src="https://img.shields.io/twitter/follow/sparkfun.svg?style=social&logo=twitter"
           	 alt="follow on Twitter"></a>
	
</p>

<img src="https://cdn.sparkfun.com//assets/parts/1/5/3/3/8/16474-SparkFun_Particle_Sensor_Breakout_-_MAX30101__Qwiic_-01.jpg"  align="right" width=300 alt="SparkFun Photodetector Breakout - MAX30101 (Qwiic)">

Python package for the [SparkFun Photodetector Breakout - MAX30101 (Qwiic)](https://www.sparkfun.com/products/16474)

This package is a port of the existing [SparkFun MAX3010x Sensor Arduino Library](https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library)

This package can be used in conjunction with the overall [SparkFun qwiic Python Package](https://github.com/sparkfun/Qwiic_Py)

New to qwiic? Take a look at the entire [SparkFun qwiic ecosystem](https://www.sparkfun.com/qwiic).

## Contents

* [Supported Platforms](#supported-platforms)
* [Dependencies](#dependencies)
* [Installation](#installation)
* [Documentation](#documentation)
* [Examples](#examples)

Supported Platforms
--------------------
The qwiic Python package current supports the following platforms:
* [Raspberry Pi](https://www.sparkfun.com/search/results?term=raspberry+pi)
* [NVidia Jetson Nano](https://www.sparkfun.com/products/15297)
* [Google Coral Development Board](https://www.sparkfun.com/products/15318)

Dependencies
================
This driver package depends on the qwiic I2C driver: 
[Qwiic_I2C_Py](https://github.com/sparkfun/Qwiic_I2C_Py)

Documentation
-------------
The SparkFun qwiic Max3010x module documentation is hosted at [ReadTheDocs](https://qwiic-max3010x-py.readthedocs.io/en/latest/index.html)

Installation
--------------

### PyPi Installation
This repository is hosted on PyPi as the [sparkfun-qwiic-max3010x](https://pypi.org/project/sparkfun-qwiic-max3010x/) package. On systems that support PyPi installation via pip, this library is installed using the following commands

For all users (note: the user must have sudo privileges):
```sh
sudo pip install sparkfun-qwiic-max3010x
```
For the current user:

```sh
pip install sparkfun_qwiic_max3010x
```

### Local Installation
To install, make sure the setuptools package is installed on the system.

Direct installation at the command line:
```sh
python setup.py install
```

To build a package for use with pip:
```sh
python setup.py sdist
 ```
A package file is built and placed in a subdirectory called dist. This package file can be installed using pip.
```sh
cd dist
pip install sparkfun_max3010x-<version>.tar.gz
```
  
Example Use
------------
See the [examples directory](#examples) for more detailed use examples.

```python

from __future__ import print_function
import qwiic_max3010x
import time
import sys

def runExample():

	print("\nSparkFun MAX3010x Photodetector Sensor - Example 1\n")
	sensor = qwiic_max3010x.QwiicMax3010x()

	if sensor.begin() == False:
		print("The Qwiic MAX3010x device isn't connected to the system. Please check your connection", \
			file=sys.stderr)
		return
	else:
		print("The Qwiic MAX3010x is connected.")

	if sensor.setup() == False:
		print("Device setup failure. Please check your connection", \
			file=sys.stderr)
		return
	else:
		print("Setup complete.")        

	while True:
			print(\
			 'R[', sensor.getRed() , '] \t'\
             'IR[', sensor.getIR() , '] \t'\
             'G[', sensor.getGreen() , ']'\
			)
			time.sleep(0.1)

if __name__ == '__main__':
	try:
		runExample()
	except (KeyboardInterrupt, SystemExit) as exErr:
		print("\nEnding Example 1")
		sys.exit(0)
```

<p align="center">
<img src="https://cdn.sparkfun.com/assets/custom_pages/3/3/4/dark-logo-red-flame.png" alt="SparkFun - Start Something">
</p>
