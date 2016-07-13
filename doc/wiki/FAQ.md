# FAQ


## When compiling toaster, I have an error: SDL library missing

If you have the error message: `Could NOT find SDL (missing: SDL_LIBRARY SDL_INCLUDE_DIR)`  
it means you should install SDL library.

toaster_simu is using SDL to position entity with the keyboard.
Please install SDL to proceed.

``` shell
> sudo apt-get install libsdl1.2-dev
```
You can also visit the [sdl installation page](https://ensiwiki.ensimag.fr/index.php/Projet_C_-_IG_-_Installation_de_SDL).

Alternatively, if you don't wish to use toaster_simu you can just black list it in catkin when compiling:
``` shell
> catkin_make -DCATKIN_BLACKLIST_PACKAGES="toaster_simu"
```


## I have some errors while compiling toaster, `* was not declared in this scope`

Please update toaster-lib:
```shell
> cd myPath/toaster-lib
> git pull
> cd build
> make install
```

If you still have errors, please send a mail to the mailing list toaster-users@laas.fr

## I'm using indigo and the 3d models in rviz are white.

This is an issue with the last version of rviz. A work around consist in installing hydro version.

```shell
> cd ${CATKIN_PATH}/src
> git clone -b hydro-devel https://github.com/ros-visualization/rviz.git
> cd ..
> catkin_make
```

## When using _agent_monitor_, nothing happens.

It may be due to boost conflict. See [this topic](http://answers.ros.org/question/173940/compiling-with-dynamic_reconfigure-server-code-leads-to-deadlock/) for details.

Two solutions are possible: use toaster-hydro branch of toaster or go in `toaster/agent_monitor/src/main.cpp` and comment lines:

```javascript
ParamServer_t monitoring_dyn_param_srv;
monitoring_dyn_param_srv.setCallback(boost::bind(&dynParamCallback, _1, _2));
```