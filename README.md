## Compile
Create a build folder, and run cmake and make in that folder
```
mkdir build
cd build
cmake ..
make
```


## Run
Run the `detect_plane` executable in the build folder. There are several arguments; the first is the path to the input pointcloud. For the remaining arguments, look in the `main` function in [plane_detection.cpp](src/plane_detection.cpp).

```
./detect_plane <path to .pcd file> 0 1.0 -0.5 0.5 0.01 0.02 50 5000
```

## Example pointcloud
Example pointclouds can be found [here](https://bib-cloud.bib.hochschule-bonn-rhein-sieg.de/apps/files/?dir=/b-it-bots-ds&fileid=3885830)
