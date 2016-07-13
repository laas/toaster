# How to install TOASTER?
## Install toaster-lib

To install toaster, you need first to install toaster-lib.
To do so, go to your installation folder, and do :

```shell
> git clone https://github.com/Greg8978/toaster-lib.git
> cd toaster-lib
> mkdir build
> cd build
> cmake ..
```

_Note: You may have to specify the boost headers directory, and you may want to specify the prefix for the install path. To do so, you can either use the gui with the command ccmake ... Press "t" key to toggle._
_Once you finished configuring, quit the cmake gui by generating the files. To do so, press "c" then "g" key._
_Alternatively you can directly specify this with the command:_

```shell
> cmake -DCMAKE_INSTALL_PREFIX=MY_INSTALL_PREFIX -DBoost_INCLUDE_DIR=MY_BOOST_INCLUDE_PATH/include ..
> make install
```

After installing `toaster-lib`, you will need to add `TOASTERLIB_DIR` variable to your `env`:

```shell
> export TOASTERLIB_DIR=MY_INSTALL_PREFIX
```

Alternatively, you may want to put this variable into your `.bashrc` 

## Install toaster

Once you installed toaster-lib, you will need to install toaster.
To do so, go to your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
You may need first to get catkin [cmake_module](http://wiki.ros.org/cmake_modules).  
You may also need to install SDL library:

```shell
> sudo apt-get install libsdl1.2-dev
```

Then use the following commands :

```shell
> cd ${CATKIN_PATH}/src
> git clone https://github.com/Greg8978/toaster.git
> cd ..
> catkin_make
```