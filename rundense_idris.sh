echo "Building Project"
cd build
make -j8

echo "Running Example"
cd ..
./Examples/Monocular/mono_dense Vocabulary/ORBvocidris.bin Examples/Monocular/tobi.yaml Video/fullstream1.mp4