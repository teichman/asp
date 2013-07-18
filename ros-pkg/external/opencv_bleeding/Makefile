built: source
	cd source/OpenCV-2.4.0 && mkdir -p release
	cd source/OpenCV-2.4.0/release && cmake -D CMAKE_BUILD_TYPE=RELEASE -D BUILD_PYTHON_SUPPORT=ON ../
	cd source/OpenCV-2.4.0/release && make -j`cat /proc/cpuinfo  | grep processor | wc -l`

source:
	mkdir -p $@
	cd $@ && wget http://downloads.sourceforge.net/project/opencvlibrary/opencv-unix/2.4.0/OpenCV-2.4.0.tar.bz2
	cd $@ && tar xvjf OpenCV-2.4.0.tar.bz2

clean:
	-cd source/OpenCV-2.4.0/release && make clean

wipe: 
	-rm -rf source
