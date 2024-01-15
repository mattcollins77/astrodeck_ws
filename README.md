Matts notes:

On PI
git pull

docker build -t piimage .

//this runs a container and mounts the volume

docker run -it --network host -v $(pwd)/src:/home/astrodeck_ws/src  piimage