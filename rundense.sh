echo "Building Project"
cd build
make -j8

echo "Running Example"
cd ..
#./Examples/Monocular/mono_dense Vocabulary/ORBvoc.bin Examples/Monocular/OSMO.yaml Video/DJI_0128.MOV
./Examples/Monocular/mono_dense Vocabulary/ORBvoc.bin Examples/Monocular/tobi.yaml Video/fullstream.mp4



#Video/fullstream_train.mp4