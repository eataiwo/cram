# CRAMPED Docker Container 
_The Collective Robotic Additive Manufacturing Performance Evaluation Demonstrator._

## Installing Docker 

## Configuring Docker 

## CRAMPED Docker File
The docker file should already be created however if it is necessary to make edits the file is located in 
this directory. For changes to take effect it is necessary to rebuild the docker container.

## Building the docker container
Changes to the docker file require a build however parts of the docker image is cached so 
only the changes in the docker file will be rebuilt. To build the docker container run the following 
command. The _tag_ flag adds the name of the container.  

``` 
docker build --tag ros-cram .
```

If you want to do a clean build e.g. you want to pull new code from git as Docker will not be 
aware of those changes then run. 

```
docker build --tag ros-cram -no-cache .
```  

## Running the docker container 

**Worked!!!!!** with usb devices no gui. Talked to micky and getting  a gui espcailly rviz is a 
big pain so it is not worth it so this it my limit for now untill I need to implement coppeliasim 
or something.  

```
docker run --rm -it --name ros --device=/dev/ttyUSB0:/dev/ttyUSB_WIDOWX1 --device=/dev/ttyUSB_WIDOWX2:/dev/ttyUSB_WIDOWX2 ros-cram roslaunch cram_v1_moveit cram_moveit.launch 
```