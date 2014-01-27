
release/built: git
	mkdir -p ${@D}
	cd ${@D} && cmake \
	-DCMAKE_BUILD_TYPE=Release \
	../git
	cd ${@D} && make -j`cat /proc/cpuinfo  | grep processor | wc -l`  # Use as many cores as you have.
	touch $@

# Aggravatingly, if you don't enable octree, io will just not build.
release-io-only/built: git
	mkdir -p ${@D}
	cd ${@D} && cmake \
	-DCMAKE_BUILD_TYPE=Release \
	-DBUILD_common=ON \
	-DBUILD_global=ON \
	-DBUILD_io=ON \
	-DBUILD_octree=ON \
	-DBUILD_TESTS=OFF \
	-DBUILD_features=OFF \
	-DBUILD_filters=OFF \
	-DBUILD_kdtree=OFF \
	-DBUILD_keypoints=OFF \
	-DBUILD_range=OFF \
	-DBUILD_registration=OFF \
	-DBUILD_sample=OFF \
	-DBUILD_segmentation=OFF \
	-DBUILD_surface=OFF \
	-DBUILD_visualization=OFF \
	../git
	cd ${@D} && make -j`cat /proc/cpuinfo  | grep processor | wc -l`  # Use as many cores as you have.
	touch $@

# HTTPS clone is not working on some machines.  Unclear why.  Using ssh as a fallback.
git:
	@git clone https://github.com/PointCloudLibrary/pcl.git git; \
	RES=$$?; \
	sleep 1; \
	if [ $$RES -ne 0 ]; then \
	echo Using ssh fallback.; \
	git clone git@github.com:PointCloudLibrary/pcl.git git; \
	fi
	cd git && git checkout pcl-1.7.0

clean:
	cd release && make clean
	-rm release/built

wipe:
	rm -rf release git